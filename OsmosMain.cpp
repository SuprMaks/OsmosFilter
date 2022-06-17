/*
 * OsmosFilterV3.cpp
 *
 * Created: 20.04.2019 23:35:50
 * Author : Maks
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "BinaryOpr.h"
#include "Vector.h"
#include "SimpleSensor.h"
#include "Delays.h"
#include "iopins.h"
#include "pinlist.h"
#include <avr/wdt.h>
#include "Common.h"
#include "MedianFilter.h"
#include "math.h"
#include "atomic.h"
#include "overflow_limit.h"
#include "Button.h"

 //FUSES = {
 //	.low = FUSE_CKSEL0, // 0xFE,
 //	.high = FUSE_BOOTRST & FUSE_SPIEN & FUSE_BOOTSZ1 & FUSE_BOOTSZ0, // 0xD8,
 //	.extended = FUSE_BODLEVEL2 // 0xFB
 //};

 //#define DEBUG 1

constexpr unsigned char watchdog = 8;
constexpr unsigned char wd_calls = 7;

constexpr auto sensor_tick_rate = 1.0 / ((double)wd_calls / watchdog);

template<typename outT = unsigned short int, typename T>
constexpr outT sec_conv(T val) {
	return (outT) simple_round(((double)val * sensor_tick_rate) + 1);
};

namespace delay {
	#ifdef DEBUG
		#define DEBUG_LEVEL 1
	#else
		namespace threshold {
			namespace high {
				constexpr auto on = sec_conv(4);
				constexpr auto off = sec_conv(2);
			}

			namespace low {
				constexpr auto on = sec_conv(4);
				constexpr auto off = sec_conv(2);
			}

			namespace valve {
				constexpr auto on = 0;
				constexpr auto off = sec_conv(2);
			}

			namespace lack {
				constexpr auto on = sec_conv(2);
				constexpr auto off = sec_conv(4);

				constexpr auto without_water = sec_conv(20 * 60);
				constexpr auto waiting_water = sec_conv(60 * 60);

				constexpr auto try_water = sec_conv(30);
				constexpr auto try_water_max = try_water * (0b1 << 6);
			}
		}
		constexpr auto WASH_BASE = 20u;

		constexpr auto PRE_WASH = sec_conv(4);
		constexpr auto FAST_WASH = sec_conv(WASH_BASE);
		constexpr auto WASH = sec_conv(WASH_BASE * 2);
		constexpr auto AUTO_WASH_LIMIT = sec_conv(WASH_BASE * 2 * 5);
		constexpr auto POST_WASH = sec_conv(4);

		constexpr auto PRE_FILLING = sec_conv(4);
		constexpr auto FINISHING = sec_conv(4);

		constexpr auto MAX_PUMPING = sec_conv(3 * 60 * 60);

		constexpr auto POST_FILL_WASH = sec_conv(WASH_BASE / 2.5);

		constexpr auto FAST_WASH_PERIOD = sec_conv(60 * 60);
		constexpr auto AUTO_WASH_PERIOD = sec_conv(6 * 60 * 60);

		constexpr auto BTN_TIMER = sec_conv(2);
		constexpr auto BTN_LONG_TIMER = sec_conv(8);
	#endif // DEBUG
}

namespace pin {
	using namespace Mcucpp::IO;

	using HighSensor = Pd0;
	using WaterLack = Pd1;
	using ValveOpened = Pd2;
	using LowSensor = Pd3;

	using Pump = Pd4;
	using Valve = Pb5;
	using WashValve = Pb4;

	using Btn = Pb0;
}

namespace reg {
	using eimsk = IOReg<0x1D + __SFR_OFFSET>;
	using eifr = IOReg<0x1C + __SFR_OFFSET>;
	using wdtcsr = IOReg<0x60>;
	using pcmsk0 = IOReg<0x6B>;
	using pcifr = IOReg<0x1B + __SFR_OFFSET>;
}

namespace flag {
	namespace wdtcsr {
		using wdif = Flag<reg::wdtcsr, WDIF>;
	}

	namespace eifr {
		using intf0 = Flag<reg::eifr, INTF0>;
		using intf1 = Flag<reg::eifr, INTF1>;
		using intf2 = Flag<reg::eifr, INTF2>;
		using intf3 = Flag<reg::eifr, INTF3>;
	}

	namespace eimsk {
		using int0 = Flag<reg::eimsk, INT0>;
		using int1 = Flag<reg::eimsk, INT1>;
		using int2 = Flag<reg::eimsk, INT2>;
		using int3 = Flag<reg::eimsk, INT3>;
	}

	namespace pcmsk0 {
		using pcint0 = Flag<reg::pcmsk0, PCINT0>;
	}

	namespace pcifr {
		using pcif0 = Flag<reg::pcifr, PCIF0>;
	}
}

namespace sensor {
	namespace reg {
		namespace low {
			using eimsk = flag::eimsk::int0;
			using eifr = flag::eifr::intf0;
			//using wdtcsr = flag::wdtcsr::wdif;
		}

		namespace high {
			using eimsk = flag::eimsk::int1;
			using eifr = flag::eifr::intf1;
			//using wdtcsr = flag::wdtcsr::wdif;
		}

		namespace valve {
			using eimsk = flag::eimsk::int2;
			using eifr = flag::eifr::intf2;
			//using wdtcsr = flag::wdtcsr::wdif;
		}

		namespace lack {
			using eimsk = flag::eimsk::int3;
			using eifr = flag::eifr::intf3;
			//using wdtcsr = flag::wdtcsr::wdif;
		}
	}
	
	namespace classes {
		using namespace pin;

		using High = Sensor<HighSensor, unsigned char, delay::threshold::high::on, delay::threshold::high::off>;
		using Low = Sensor<LowSensor, unsigned char, delay::threshold::low::on, delay::threshold::low::off>;

		class Valve : public Sensor<ValveOpened, unsigned char, delay::threshold::valve::on, delay::threshold::valve::off> {
			public:
				class Stats {
					public:
						bool freeze = true;

					private:
						friend Valve;

						MedianFilteredValue <unsigned short int, 16> current;
						unsigned short int staging;

						unsigned short int inline get_staging(void) {
							volatile unsigned short int tmp;
							ATOMIC{
								tmp = staging;
								memory();
								staging = 0;
							}
							memory();
							return tmp;
						}

						void inline __attribute__((always_inline)) timerCallback(void) {
							if (!freeze) {
								overflow_limit_sum(staging, 1);
							}
						}

					public:
						Stats(void) : staging(0) {}

						unsigned short int inline __attribute__((always_inline)) operator()(void) {
							return current();
						}
				} stats;

			private:
				typedef Sensor<ValveOpened, unsigned char, delay::threshold::valve::on, delay::threshold::valve::off> base;

			public:
				Valve(void) : base() {}

				void inline __attribute__((always_inline)) operator()(void) override {
					base::operator()();
					memory();
					if (value()) {
						stats.timerCallback();
					}
				}

				inline operator bool(void) {
					unsigned short int tmp = stats.operator()();
					volatile unsigned short int tmp_stg;
					if (tmp > 0) {
						ATOMIC{
							tmp_stg = stats.staging;
						}
						return tmp <= tmp_stg;
					}
					return false;
				}

				void inline __attribute__((always_inline)) stats_new_iteration() {
					stats.current(stats.get_staging());
				}
		};

		class WaterLack : public Sensor<pin::WaterLack, unsigned int, delay::threshold::lack::on, delay::threshold::lack::off> {
			protected:
				typedef Sensor<pin::WaterLack, unsigned int, delay::threshold::lack::on, delay::threshold::lack::off> base;
				friend ISRf(INT3_vect);

				enum class WaterLackMode : const unsigned char {
					Normal,
					Lost,
					Waiting,
				};

				unsigned int lost_timer;

				WaterLackMode mode;

				unsigned int get_timer_delay(const bool state) override {
					if (mode == WaterLackMode::Waiting) {
						return delay::threshold::lack::waiting_water;
					}
					return base::get_timer_delay(state);
				}

				void inline __attribute__((always_inline)) handler_timer_done(void) override {
					if (stage_val != value()) {
						if (stage_val) {
							mode = WaterLackMode::Normal;
						}
						else {
							mode = WaterLackMode::Lost;
							this->time = delay::threshold::lack::without_water;
							memory();
						}
						base::handler_timer_done();
					}
					else if(!value() && mode == WaterLackMode::Lost) {
						mode = WaterLackMode::Waiting;
					}
				}

			public:
				WaterLack(void) : base(), mode(real_val() ? WaterLackMode::Normal : WaterLackMode::Lost) {};

		};
	}

	classes::High High;
	classes::Low Low;
	classes::Valve Valve;
	classes::WaterLack WaterLack;
}

namespace button {
	namespace reg {
		namespace btn {
			using msk = flag::pcmsk0::pcint0;
			using flg = flag::pcifr::pcif0;
			//using timer_flg = flag::wdtcsr::wdif;
		}
	}

	namespace classes {
		using Btn = ButtonAssync<pin::Btn, unsigned char, reg::btn::msk, reg::btn::flg>;
	}

	classes::Btn Btn;
}

class Pump : public Timer<unsigned short int> {
	private:
		typedef unsigned short int TimerDT;
		typedef Timer<TimerDT> base;
		TimerDT timer_value;

	public:
		enum State : const unsigned char {
			Ready,
			OverPumping,
			Cooldown,
			Working
		};

	protected:
		#ifdef WDT_vect
			friend ISRf(WDT_vect);
		#endif
		friend int main();

		MedianFilteredValue <TimerDT, 8> stat;

		TimerDT _incr_delay;

		TimerDT inline __attribute__((always_inline)) incr_delay(void) {
			if (this->_state == State::Cooldown || this->_state == State::Ready) {
				TimerDT tmp;
				ATOMIC{
					tmp = _incr_delay;
				}
				return tmp;
			}
			return _incr_delay;
		}

		void inline __attribute__((always_inline)) handler_timer_done(void) override {
			switch (this->_state) {
				case State::Working:
					this->_state = State::OverPumping;
					break;

				case State::Cooldown:
					this->_state = State::Ready;
				case State::Ready:
					if (_incr_delay > delay::threshold::lack::try_water) {
						if (atomic) {
							_incr_delay /= 2;
						}
						else {
							atomic = true;
						}
						this->_timer(_incr_delay);
					}

					break;
			}
		}

		TimerDT pump_time(void) {
			TimerDT tmp = delay::MAX_PUMPING;
			if (stat() != 0) {
				tmp = (TimerDT)simple_round((double)stat() * 1.3);
				if (tmp > delay::MAX_PUMPING) {
					tmp = delay::MAX_PUMPING;
				}
			}
			return tmp;
		}

		State _state;

	public:
		bool atomic_time, atomic;

		void timer(const TimerDT delay) override {
			base::timer(delay);
			timer_value = delay;
		}

		TimerDT inline elapsed_time(void) {
			return timer_value - base::timer();
		}

		void inline __attribute__((always_inline)) reset_timer(void) {
			this->timer(pump_time());
		}

		void new_iteration(void) {
			if (atomic_time) {
				stat(elapsed_time());
			}
		}
		
		State inline __attribute__((always_inline)) state(void) {
			return _state;
		}

		void inline __attribute__((always_inline)) state(State st) {
			memory();
			_state = st;
			memory();
		}

		Pump(void) : base(), _state(State::Ready), _incr_delay(delay::threshold::lack::try_water), atomic_time(true), atomic(true) {
			pin::Pump::SetConfiguration(pin::Pump::Port::Out);
			pin::Pump::Clear();
		}

		void on(void) {
			if (!pin::Pump::IsSet()) {
				pin::Pump::Set();
				memory();
				state(State::Working);
				memory();
				this->reset_timer();
			}
		};

		void off(void) {
			if (pin::Pump::IsSet()) {
				pin::Pump::Clear();
				memory();
				state(State::Cooldown);
				memory();

				if (atomic) {
					TimerDT sum = 0;
					while (incr_delay() > delay::threshold::lack::try_water && elapsed_time() > sum) {
						sum += incr_delay();
						ATOMIC{
							_incr_delay /= 2;
						}
					}
					this->timer(incr_delay());
				}
				else {
					this->timer(incr_delay());
					if (incr_delay() < delay::threshold::lack::try_water_max) {
						ATOMIC{
							_incr_delay *= 2;
						}
					}
				}
			}
		}
} pump;

class Tank {
	public:
		enum class State :const unsigned char {
			Empty = 0x03,
			Normal = 0x02,
			Impossible = 0x01,
			Full = 0,
		};

		static State state(void) {
			static State buffer;
			const State state = static_cast <State>((unsigned char)(((sensor::High.value() ? 0x01u : 0x00u) << 1) | (sensor::Low.value() ? 0x01u : 0x00u)));

			memory();

			if (state == State::Impossible) {
				exit(EXIT_SUCCESS);
			}
			else if (state == State::Full) {
				buffer = state;
			}
			else if (!sensor::Valve.value()) {
				buffer = state;
			}
			else if (state == State::Empty && sensor::Valve) {
				buffer = state;
			}
			return buffer;
		};
};

Timer<unsigned short int> timer(0xff);

namespace wd {
	callback* const callbacks[]{ &sensor::High, &sensor::Low, &sensor::Valve, &sensor::WaterLack, &pump, &timer, &button::Btn };
	constexpr unsigned char const count = (sizeof(callbacks) / sizeof(callbacks[0])) - 1;
	unsigned char counter = 0;
}

enum class State : const unsigned char {
	STANDBY,

	PRE_WASH,
	WASH,
	POST_WASH,

	PRE_FILL,
	FILL,

	POST_FILL_WASH,
	DONE,

	FINISHING,

	AUTO_WASH,
};

static State state = State::STANDBY;

enum class delaysID : const unsigned char {
	PRE_WASH,
	WASH,
	POST_WASH,
	PRE_FILL,
	POST_FILL_WASH,
	FINISHING,
	FAST_WASH_PERIOD,
};

namespace task {
	class Task {
		protected:
			enum class BtnState : const unsigned char {
				FREE,
				PRESS,
				CLICK,
				LONG_PRESS,
			};

			State virtual inline constexpr task(void) = 0;

			BtnState btn_state(void) {
				const auto tmp = button::Btn.timer();
				if (tmp > delay::BTN_LONG_TIMER) {
					return BtnState::LONG_PRESS;
				}
				else if (!button::Btn) {
					if (tmp > delay::BTN_TIMER) {
						return BtnState::PRESS;
					}
					else if (button::Btn.clicks() != 0) {
						return BtnState::CLICK;
					}
				}
				return BtnState::FREE;
			}

			static inline bool interrupted = false;
			static inline bool ignore_lack_sensor = false;

			void inline check_btn_state(void) {
				if (ignore_lack_sensor && BtnState::CLICK == btn_state()) {
					ignore_lack_sensor = false;
					button::Btn.disable();
				}
			}
			
			static bool wLack(void) {
				const bool tmp = !sensor::WaterLack.value();
				if (tmp) {
					interrupted = true;
					if (!ignore_lack_sensor) {
						state = State::FINISHING;
					}
				}
				return tmp && !ignore_lack_sensor;
			}

		public:
			virtual operator bool(void) = 0;

			void inline __attribute__((always_inline)) operator()(void) {
				for (; *this;) {
					sleep_mode();
				}
			}
	};

	class WashTask : public Task {
		public:
			enum class State : const unsigned char {
				AUTO,
				FAST,
				FULL
			};

			static inline State state = State::FULL;

			static void inline __attribute__((always_inline)) increment_skipped_autowashes(void) {
				if (skipped_autowashes < (const unsigned char)simple_round((double)delay::AUTO_WASH_LIMIT / delay::WASH)) {
					skipped_autowashes++;
				}
			}

			static bool inline __attribute__((always_inline)) is_skipped_autowashes(void) {
				return skipped_autowashes > 1;
			}

		protected:
			static inline unsigned char skipped_autowashes = 1;
 
			void inline check_btn_state(void) {
				if (state != State::AUTO) {
					Task::check_btn_state();
				}
			}

			static bool wLack(void) {
				if (state == State::AUTO) {
					bool tmp = !sensor::WaterLack.value();
					if (tmp) {
						::state = ::State::FINISHING;
					}
					return tmp;
				}
				else {
					return Task::wLack();
				}
			}
	};

	class Stanby : public Task {
		private:
			unsigned char auto_wash_period_encounter;

		protected:
			State virtual inline constexpr task(void) override {
				return State::STANDBY;
			}

		public:
			Stanby(void) : auto_wash_period_encounter(0) {
				pump.off();
				memory();
				pin::Valve::Clear();
				memory();
				pin::WashValve::Clear();
				ignore_lack_sensor = false;
				
				button::Btn.reset();
				memory();
				button::Btn.enable();
				WashTask::state = WashTask::State::FAST;
				set_sleep_mode(SLEEP_MODE_STANDBY);
			}

			~Stanby(void) {
				sensor::Valve.stats.freeze = true;
				state = State::PRE_WASH;

				if (ignore_lack_sensor) {
					button::Btn.reset();
				}
				else {
					button::Btn.disable();
				}
				set_sleep_mode(SLEEP_MODE_IDLE);
			}

			operator bool(void) override {
				const Tank::State tmp = Tank::state();
				const bool lck = sensor::WaterLack.value();

				if (tmp != Tank::State::Full) {
					switch (btn_state()) {
						case BtnState::LONG_PRESS:
							ignore_lack_sensor = true;
						case BtnState::PRESS:
							if (!lck && !sensor::WaterLack.val_in_sync()) {
								sensor::WaterLack.timer(0);
							}
						case BtnState::CLICK:
							if (tmp != Tank::State::Empty) {
								interrupted = true;
							}
							pump.state(pump.Ready);
					}
				}
				
				bool auto_wash = WashTask::is_skipped_autowashes();
				if (timer.isDone()) {
					if (auto_wash_period_encounter < (const unsigned char) simple_round((double)delay::AUTO_WASH_PERIOD / delay::FAST_WASH_PERIOD)) {
						WashTask::state = WashTask::State::FULL;
						auto_wash_period_encounter++;
						timer.timer(delay::FAST_WASH_PERIOD);
					}
					else {
						auto_wash = true;
						if (!lck) {
							WashTask::increment_skipped_autowashes();
							auto_wash_period_encounter = 0;
							timer.timer(delay::FAST_WASH_PERIOD);
						}
					}
				}

				memory();

				if (((tmp == Tank::State::Empty ||
					(tmp != Tank::State::Full && interrupted)) &&
					(lck || ignore_lack_sensor) && pump.state() == pump.Ready)) {
					return false;
				}
				else if (auto_wash && lck) {
					WashTask::state = WashTask::State::AUTO;
					return false;
				}
				return true;

				/*return !((tmp == Tank::State::Empty ||
					(tmp != Tank::State::Full && interrupted)) &&
					(sensor::WaterLack.value() || ignore_lack_sensor) && pump.state() == pump.Ready);*/
			}
	};

	class PreWash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::PRE_WASH;
			}

		public:
			PreWash(void) {
				//pump.off();
				pin::WashValve::Set();
				memory();
				pin::Valve::Set();
				//pin::WashValve::Clear();
			}

			/*~PreWash(void) {
				if (state == FINISHING) {
					pin::WashValve::Clear();
				}
			}*/

			operator bool(void) override {
				check_btn_state();

				/*if (state == State::AUTO) {
					if (_wLack()) {
						return false;
					}
				}
				else */if (wLack()) {
					//pin::WashValve::Clear();
					return false;
				}
				else if (timer((const unsigned char)delaysID::PRE_WASH, delay::PRE_WASH)) {
					::state = ::State::WASH;
					return false;
				}

				return true;
			}
	};

	class Wash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::WASH;
			}

		public:
			Wash(void) {
				//pin::Valve::Set();
				//pin::WashValve::Set();
				pin::Pump::Set();
			}

			//~Wash(void) {
			//	//pin::WashValve::Clear();
			//	if (state == POST_WASH) {
			//		
			//	}
			//	

			//}

			operator bool(void) override {
				check_btn_state();

				/*if (state == State::AUTO) {
					if (_wLack()) {
						pump.off();
						return false;
					}
				}
				else*/ if (wLack()) {
					pump.off();
					return false;
				}
				else if (timer((const unsigned char)delaysID::WASH, state == State::FAST ? delay::FAST_WASH : ((delay::WASH - 1) * skipped_autowashes) + 1)) {
					::state = ::State::POST_WASH;
					skipped_autowashes = 1;
					pin::Pump::Clear();
					return false;
				}

				return true;
			}
	};

	class PostWash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::POST_WASH;
			}

		public:
			~PostWash(void) {
				//pin::WashValve::Clear();
				pin::WashValve::Clear();
			}

			operator bool(void) override {
				check_btn_state();

				/*if (state == State::AUTO) {
					if (_wLack()) {
						return false;
					}
				}
				else*/ if (wLack()) {
					return false;
				}
				else if (timer((const unsigned char)delaysID::POST_WASH, delay::POST_WASH)) {
					if (state == State::AUTO) {
						::state = ::State::FINISHING;
					}
					else {
						::state = ::State::PRE_FILL;
					}
					return false;
				}

				return true;
			}
	};

	class PreFill : public Task {
		protected:
			State virtual inline constexpr task(void) override {
				return State::PRE_FILL;
			}

		public:
			PreFill(void) {
				pin::Valve::Set();
			}

			//~PreFill(void) {
			//	state = FILL;
			//}

			operator bool(void) override {
				check_btn_state();

				if (wLack()) {
					return false;
				}
				else if (timer((const unsigned char)delaysID::PRE_FILL, delay::PRE_FILLING)) {
					state = State::FILL;
					return false;
				}

				return true;
			}
	};

	class Fill : public Task {
		protected:
			State virtual inline constexpr task(void) override {
				return State::FILL;
			}

		public:
			Fill(void) {
				pump.atomic_time = !interrupted;
				memory();
				pump.on();
			}

			~Fill(void) {
				if (!interrupted) {
					pump.new_iteration();
				}
				memory();
				if (state == State::DONE) {
					interrupted = false;
					state = State::FINISHING;
					sensor::Valve.stats.freeze = false;
				}
				else {
					pump.atomic = false;
				}
				memory();
				pump.off();
			}

			operator bool(void) override {
				check_btn_state();

				if (state == State::POST_FILL_WASH) {
					if (timer((const unsigned char)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
						state = State::DONE;
						return false;
					}
					else if (wLack()) {
						return false;
					}
				}
				else {
					if (Tank::state() == Tank::State::Full) {
						pin::WashValve::Set();
						state = State::POST_FILL_WASH;
					}
					else if (wLack()) {
						return false;
					}
					else {
						if (pump.state() == pump.OverPumping) {
							exit(EXIT_SUCCESS);
							return false;
						}
						else if (sensor::Valve.value()) {
							pump.atomic_time = false;
							memory();
							pump.reset_timer();
						}
					}
				}
				return true;
			}
	};

	class PostFillWash : public Task {
		protected:
			State virtual inline constexpr task(void) override {
				return State::POST_FILL_WASH;
			}

		public:
			PostFillWash(void) {
				pin::WashValve::Set();
			}

			~PostFillWash(void) {
				pump.off();
			}

			operator bool(void) override {
				check_btn_state();

				if (wLack() || timer((const unsigned char)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
					state = State::FINISHING;
					return false;
				}
				return true;
			}
	};

	class Finishing : public Task {
		protected:
			State virtual inline constexpr task(void) override {
				return State::FINISHING;
			}

		public:
			//Finishing(void) {
			//	/*pin::Valve::Set();*/
			//	pin::WashValve::Clear();
			//}

			~Finishing(void) {
				pin::Valve::Clear();
				memory();
				pin::WashValve::Clear();

				if (!interrupted) {
					sensor::Valve.stats_new_iteration();
				}

				state = State::STANDBY;
				timer((const unsigned char)delaysID::FAST_WASH_PERIOD, delay::FAST_WASH_PERIOD);
			}

			operator bool(void) override {
				return !timer((const unsigned char)delaysID::FINISHING, delay::FINISHING);
			}
		};
}

template<typename Port, unsigned char mask = 0xff>
void inline __attribute__((always_inline)) hizPort(void) {
	memory();
	Port::SetConfiguration(mask, Port::In);
	memory();
	Port::SetPullUp(mask, Port::PullUp);
	memory();
}

void __attribute__((used, naked, section(".init1")))
pre_init(void) {
	MCUCR = 0x00;

	pin::Valve::SetConfiguration(pin::Valve::Port::Out);
	pin::Valve::Clear();

	pin::WashValve::SetConfiguration(pin::WashValve::Port::Out);
	pin::WashValve::Clear();
	memory();

	#ifdef DEBUG
		bitSet(DDRB, PINB0);
		bitSet(DDRD, PIND5);
	#else
		//hizPort<Mcucpp::IO::Portb, 0xCF>();
		hizPort<Mcucpp::IO::Portb, 0xCE>();
		hizPort<Mcucpp::IO::Portd, 0xE0>();
		hizPort<Mcucpp::IO::Portc>();
		hizPort<Mcucpp::IO::Porte>();
		hizPort<Mcucpp::IO::Portf>();
	#endif // DEBUG

	bitClear(ADCSRA, ADEN);
	bitSet(ACSR, ACD);

	PRR0 = bit(PRTWI) | bit(PRTIM0) | bit(PRTIM1) | bit(PRSPI) | bit(PRADC);
	//PRR1 = bit(PRUSB) | bit(PRTIM3) | bit(PRUSART1);
	PRR1 = bit(PRUSB) | bit(PRTIM4) | bit(PRTIM3) | bit(PRUSART1);
}

void __attribute__((used, naked, section(".init8")))
init(void) {
	wdt_reset(); //asm("wdr");
	memory();
	WDTCSR |= bit(WDCE) | bit(WDE);
	memory();
	WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1); //bit(WDP2);
	memory();

	// Enable pin change interrupt for button
	PCICR = bit(PCIE0);
	memory();
	PCIFR = 0;
	memory();
	PCMSK0 = bit(PCINT0);

	// Enable Interrupt any edge
	EICRA = bit(ISC30) | bit(ISC20) | bit(ISC10) | bit(ISC00);
	memory();
	EIFR = 0;
	memory();
	EIMSK = bit(INT0) | bit(INT1) | bit(INT2) | bit(INT3);
	

	sei();
	set_sleep_mode(SLEEP_MODE_STANDBY);
}

void __attribute__((used, naked, section(".fini0")))
terminate(void) {
	cli();
	pump.off();
	pin::Valve::Clear();
	pin::WashValve::Clear();
	memory();
	sleep_mode();
}

__attribute__((OS_main))
int main(void) {
	for (;;) {
		#ifdef DEBUG
			bitWrite(PORTB, PINB0, Tank::state() == Tank::Empty);
			bitWrite(PORTD, PIND5, sensor::High.value() && sensor::Low.value());
		#endif // DEBUG

		switch (state) {
			case State::STANDBY:
				task::Stanby()();
				break;

			case State::PRE_WASH:
				task::PreWash()();
				break;

			case State::WASH:
				task::Wash()();
				break;

			case State::POST_WASH:
				task::PostWash()();
				break;

			case State::PRE_FILL:
				task::PreFill()();
				break;

			case State::FILL:
				task::Fill()();
				break;

			//case States::POST_FILL_WASH:
			//	task::PostFillWash()();
			//	break;

			case State::FINISHING:
				task::Finishing()();
				break;
		}
	}
}

ISRf(INT0_vect) {
	sensor::High.interruptCallback();
}

ISRf(INT1_vect) {
	sensor::WaterLack.interruptCallback();
}

ISRf(INT2_vect) {
	sensor::Valve.interruptCallback();
	memory();
	pump.atomic_time = false;
}

ISRf(INT3_vect) {
	sensor::Low.interruptCallback();
}

ISRf(PCINT0_vect) {
	button::Btn.interruptCallback();
}

ISRf(WDT_vect) {
	bitSet(WDTCSR, WDIE);
	memory();
	wd::callbacks[wd::counter]->operator()();
	memory();
	if (wd::counter < wd::count) {
		wd::counter++;
	}
	else {
		wd::counter = 0;
	}
	memory();
	bitSet(WDTCSR, WDIE);
}