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
				constexpr auto on = sec_conv(1);
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
		constexpr auto PRE_WASH = sec_conv(5);
		constexpr auto FAST_WASH = sec_conv(20);
		constexpr auto WASH = sec_conv(50);
		constexpr auto POST_WASH = sec_conv(5);

		constexpr auto PRE_FILLING = sec_conv(5);
		constexpr auto POST_FILLING = sec_conv(5);

		//constexpr auto BETWEEN_FILLING = sec_conv(60);
		constexpr auto MAX_PUMPING = sec_conv(3 * 60 * 60);

		constexpr auto POST_FILL_WASH = sec_conv(15);

		constexpr auto WASH_TIMER = sec_conv(60 * 60);
		constexpr auto AUTO_WASH_PERIOD = sec_conv(6 * 60 * 60);

		constexpr auto BTN_TIMER = sec_conv(2);
		constexpr auto BTN_LONG_TIMER = sec_conv(8);
	#endif // DEBUG
}

namespace pins {
	using namespace Mcucpp::IO;

	using HighSensor = Pd0;
	using WaterLack = Pd1;
	using ValveOpened = Pd2;
	using LowSensor = Pd3;

	/*using LowSensor = Pd0;
	using HighSensor = Pd1;
	using ValveOpened = Pd2;
	using WaterLack = Pd3;*/

	using Pump = Pd4;
	using Valve = Pb5;
	using WashValve = Pb4;

	using Btn = Pb0;
}

namespace regs {
	using eimsk = IOReg<0x1D + __SFR_OFFSET>;
	using eifr = IOReg<0x1C + __SFR_OFFSET>;
	using wdtcsr = IOReg<0x60>;
	using pcmsk0 = IOReg<0x6B>;
	using pcifr = IOReg<0x1B + __SFR_OFFSET>;
}

namespace flags {
	namespace wdtcsr {
		using wdif = Flag<regs::wdtcsr, WDIF>;
	}

	namespace eifr {
		using intf0 = Flag<regs::eifr, INTF0>;
		using intf1 = Flag<regs::eifr, INTF1>;
		using intf2 = Flag<regs::eifr, INTF2>;
		using intf3 = Flag<regs::eifr, INTF3>;
	}

	namespace eimsk {
		using int0 = Flag<regs::eimsk, INT0>;
		using int1 = Flag<regs::eimsk, INT1>;
		using int2 = Flag<regs::eimsk, INT2>;
		using int3 = Flag<regs::eimsk, INT3>;
	}

	namespace pcmsk0 {
		using pcint0 = Flag<regs::pcmsk0, PCINT0>;
	}

	namespace pcifr {
		using pcif0 = Flag<regs::pcifr, PCIF0>;
	}
}

namespace sensors {
	namespace regs {
		namespace low {
			using eimsk = flags::eimsk::int0;
			using eifr = flags::eifr::intf0;
			//using wdtcsr = flags::wdtcsr::wdif;
		}

		namespace high {
			using eimsk = flags::eimsk::int1;
			using eifr = flags::eifr::intf1;
			//using wdtcsr = flags::wdtcsr::wdif;
		}

		namespace valve {
			using eimsk = flags::eimsk::int2;
			using eifr = flags::eifr::intf2;
			//using wdtcsr = flags::wdtcsr::wdif;
		}

		namespace lack {
			using eimsk = flags::eimsk::int3;
			using eifr = flags::eifr::intf3;
			//using wdtcsr = flags::wdtcsr::wdif;
		}
	}
	
	namespace classes {
		using namespace pins;

		using High = Sensor<HighSensor, unsigned char, delay::threshold::high::on, delay::threshold::high::off>;
		using Low = Sensor<LowSensor, unsigned char, delay::threshold::low::on, delay::threshold::low::off>;

		class Valve : public Sensor<ValveOpened, unsigned char, delay::threshold::valve::on, delay::threshold::valve::off> {
			public:
				class Stats {
					public:
						bool freeze = true;

					private:
						friend Valve;

						MedianFilteredValue <unsigned short int, 10> current;
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

						/*operator bool(void) {
							unsigned short int tmp = operator()();
							if (tmp > 0) {
								return tmp <= atomic_read(staging);
							}
							return false;
						}*/

						//void new_iteration(void) {
						//	volatile unsigned short int tmp = get_staging();
						//	//ATOMIC {
						//		/*tmp = staging;
						//		staging = 0;*/
						//	/*}
						//	bitSet(WDTCSR, WDIE);*/
						//	memory();
						//	current(tmp);
						//}
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

		class WaterLack : public Sensor<pins::WaterLack, unsigned int, delay::threshold::lack::on, delay::threshold::lack::off> {
			protected:
				typedef Sensor<pins::WaterLack, unsigned int, delay::threshold::lack::on, delay::threshold::lack::off> base;
				friend ISRf(INT3_vect);

				enum class WaterLackMode : const unsigned char {
					Normal,
					Lost,
					Waiting,
				};

				unsigned int lost_timer;

				WaterLackMode mode;

				const unsigned int get_timer_delay(const bool state) const override {
					/*switch (mode) {
						case WaterLackMode::Normal:
						case WaterLackMode::Lost:
							return base::get_timer_delay(state);
							break;
						case WaterLackMode::Waiting:
							return delay::threshold::lack::waiting_water;
							break;
					}*/
					if (mode == WaterLackMode::Waiting) {
						return delay::threshold::lack::waiting_water;
					}
					return base::get_timer_delay(state);
					//return !this->normal_mode & state ? delay::threshold::lack::waiting_water : base::get_timer_delay(state);
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
							//this->time_sync = false;
						}
						base::handler_timer_done();
					}
					else if(!value() && mode == WaterLackMode::Lost) {
						/*unsigned int tmp = base::get_timer_delay(value());
						if (tmp > lost_timer) {
							lost_timer = 0;
						}
						else {
							lost_timer -= tmp;
						}

						if (lost_timer == 0) {*/
							mode = WaterLackMode::Waiting;
						//}
					}
				}

				/*void handler_timer_done(void) override {
					if (this->stage_val != this->value()) {
						base::handler_timer_done();
						memory();
						this->normal_mode = true;
						memory();
						if (!this->value()) {
							this->timer(delay::threshold::lack::without_water);
						}

					}
					else if (!this->value()) {
						this->normal_mode = false;
					}
				}*/

			public:
				WaterLack(void) : base(), mode(real_val() ? WaterLackMode::Normal : WaterLackMode::Lost) {};

		};
	}

	classes::High High;
	classes::Low Low;
	classes::Valve Valve;
	classes::WaterLack WaterLack;
}

namespace buttons {
	namespace regs {
		namespace btn {
			using msk = flags::pcmsk0::pcint0;
			using flg = flags::pcifr::pcif0;
			//using timer_flg = flags::wdtcsr::wdif;
		}
	}

	namespace classes {
		using Btn = ButtonAssync<pins::Btn, unsigned char, regs::btn::msk, regs::btn::flg>;
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

		MedianFilteredValue <TimerDT, 10> stat;

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
			TimerDT tmp;
			if (stat() == 0) {
				tmp = delay::MAX_PUMPING;
			}
			else {
				tmp = (TimerDT) simple_round((double)stat() * 1.2);
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
			//atomic_wrapper_in();
			memory();
			_state = st;
			memory();
			//atomic_wrapper_out();
		}

		Pump(void) : base(), _state(State::Ready), _incr_delay(delay::threshold::lack::try_water), atomic_time(true), atomic(true) {
			pins::Pump::SetConfiguration(pins::Pump::Port::Out);
			pins::Pump::Clear();
		}

		void on(void) {
			if (!pins::Pump::IsSet()) {
				pins::Pump::Set();
				memory();
				state(State::Working);
				memory();
				this->reset_timer();
			}
		};

		void off(void) {
			if (pins::Pump::IsSet()) {
				pins::Pump::Clear();
				memory();
				state(State::Cooldown);
				memory();

				/*if (elapsed_time() < sec_conv(30)) {
					
				}
				else if (elapsed_time() > incremental_delay) {
					if (incremental_delay > delay::threshold::lack::try_water) {
						incremental_delay /= 2;
					}
				}
				this->timer(incremental_delay);*/

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
				//atomic = atomic_time;

				//this->timer((elapsed_time() < delay::threshold::lack::off + sec_conv(8)) ? delay::threshold::lack::try_water : delay::BETWEEN_FILLING);
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
			State state = static_cast <State>((unsigned char)(((sensors::High.value() ? 0x01 : 0x00) << 1) | (sensors::Low.value() ? 0x01 : 0x00)));

			if (state == State::Impossible) {
				exit(EXIT_SUCCESS);
			}
			else if (state == State::Full) {
				buffer = state;
			}
			else if (!sensors::Valve.value()) {
				buffer = state;
			}
			else if (state == State::Empty && sensors::Valve) {
				buffer = state;
			}
			return buffer;
		};
};

//Timer<unsigned short int, flags::wdtcsr::wdif> timer(0xff);
Timer<unsigned short int> timer(0xff);

namespace wd {
	callback* const callbacks[]{ &sensors::High, &sensors::Low, &sensors::Valve, &sensors::WaterLack, &pump, &timer, &buttons::Btn };
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
	POST_FILL,

	AUTO_WASH,
};

static State state = State::STANDBY;

enum class delaysID : const unsigned char {
	PRE_WASH,
	WASH,
	POST_WASH,
	PRE_FILL,
	POST_FILL_WASH,
	POST_FILL,
	WASH_TIMER,
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
				const auto tmp = buttons::Btn.timer();
				if (tmp > delay::BTN_LONG_TIMER) {
					return BtnState::LONG_PRESS;
				}
				else if (!buttons::Btn) {
					if (tmp > delay::BTN_TIMER) {
						return BtnState::PRESS;
					}
					else if (buttons::Btn.clicks() != 0) {
						return BtnState::CLICK;
					}
				}

				/*else if (tmp > delay::BTN_TIMER) {
					return BtnState::PRESS;
				}
				else if (!buttons::Btn && buttons::Btn.clicks() != 0) {
					return BtnState::CLICK;
				}*/
				return BtnState::FREE;
			}

			static inline bool interrupted = false;
			static inline bool ignore_lack_sensor = false;

			void inline check_btn_state(void) {
				if (ignore_lack_sensor && BtnState::CLICK == btn_state()) {
					ignore_lack_sensor = false;
					buttons::Btn.disable();
				}
			}
			
			static bool wLack(void) {
				bool tmp = !sensors::WaterLack.value();
				if (tmp) {
					interrupted = true;
					if (!ignore_lack_sensor) {
						state = State::POST_FILL;
					}
				}
				return tmp && !ignore_lack_sensor;
			}

		public:
			virtual operator bool(void) = 0;

			void inline __attribute__((always_inline)) operator()(void) {
				for (; *this;) {
					if (this->task() == State::STANDBY) {
						sleep_mode();
					}
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

		protected:
			void inline check_btn_state(void) {
				if (state != State::AUTO) {
					Task::check_btn_state();
				}
			}

			static bool wLack(void) {
				if (state == State::AUTO) {
					bool tmp = !sensors::WaterLack.value();
					if (tmp) {
						::state = ::State::POST_FILL;
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
			/*enum BtnState : const unsigned char {
				btn_free,
				btn_click,
				btn_long_press,
			};

			bool prev_btn;
			unsigned short int timer_pos;*/
		protected:
			State virtual inline constexpr task(void) override {
				return State::STANDBY;
			}

		public:
			Stanby(void) : auto_wash_period_encounter(0) {
				pump.off();
				memory();
				pins::Valve::Clear();
				memory();
				pins::WashValve::Clear();
				ignore_lack_sensor = false;
				//timer_pos = 0;
				
				buttons::Btn.reset();
				memory();
				buttons::Btn.enable();
				WashTask::state = WashTask::State::FAST;
			}

			~Stanby(void) {
				sensors::Valve.stats.freeze = true;

				/*if (timer.isDone()) {
					state = States::PRE_WASH;
				}
				else {
					state = States::PRE_FILL;
				}*/

				state = State::PRE_WASH;

				if (ignore_lack_sensor) {
					buttons::Btn.reset();
				}
				else {
					buttons::Btn.disable();
				}
			}

			operator bool(void) override {
				Tank::State tmp = Tank::state();
				bool lck = sensors::WaterLack.value();

				switch (btn_state()) {
					case BtnState::LONG_PRESS:
						ignore_lack_sensor = true;
						interrupted = true;
					case BtnState::PRESS:
						if (!lck && !sensors::WaterLack.val_in_sync()) {
							sensors::WaterLack.timer(0);
						}
					case BtnState::CLICK:
						if (tmp != Tank::State::Empty && tmp != Tank::State::Full) {
							interrupted = true;
						}
						pump.state(pump.Ready);
				}

				/*return !((tmp == Tank::Empty ||
					(tmp != Tank::Full && interrupted)) &&
					sensors::WaterLack.value() && pump.state() == pump.Ready);*/

				/*if (timer.isDone()) {
					if (auto_wash_period_encounter < 6) {
						WashTask::state = WashTask::State::FULL;
						auto_wash_period_encounter++;
						timer.timer(delay::WASH_TIMER);
					}
					else if (sensors::WaterLack.value()) {
						WashTask::state = WashTask::State::AUTO;
						return false;
					}
				}*/
				
				bool auto_wash = false;
				if (timer.isDone()) {
					if (auto_wash_period_encounter < (const unsigned char) simple_round((double)delay::AUTO_WASH_PERIOD / delay::WASH_TIMER)) {
						WashTask::state = WashTask::State::FULL;
						auto_wash_period_encounter++;
						timer.timer(delay::WASH_TIMER);
					}
					else if (lck) {
						auto_wash = true;
					}
				}

				if (((tmp == Tank::State::Empty ||
					(tmp != Tank::State::Full && interrupted)) &&
					(lck || ignore_lack_sensor) && pump.state() == pump.Ready)) {
					return false;
				}
				else if (auto_wash) {
					WashTask::state = WashTask::State::AUTO;
					return false;
				}
				return true;

				/*return !((tmp == Tank::State::Empty ||
					(tmp != Tank::State::Full && interrupted)) &&
					(sensors::WaterLack.value() || ignore_lack_sensor) && pump.state() == pump.Ready);*/
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
				pins::WashValve::Set();
				memory();
				pins::Valve::Set();
				//pins::WashValve::Clear();
			}

			/*~PreWash(void) {
				if (state == POST_FILL) {
					pins::WashValve::Clear();
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
					//pins::WashValve::Clear();
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
				//pins::Valve::Set();
				//pins::WashValve::Set();
				pins::Pump::Set();
			}

			//~Wash(void) {
			//	//pins::WashValve::Clear();
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
				else if (timer((const unsigned char)delaysID::WASH, state == State::FAST ? delay::FAST_WASH : delay::WASH)) {
					::state = ::State::POST_WASH;
					pins::Pump::Clear();
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
				//pins::WashValve::Clear();
				pins::WashValve::Clear();
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
						::state = ::State::POST_FILL;
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
				pins::Valve::Set();
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
					state = State::POST_FILL;
					sensors::Valve.stats.freeze = false;
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
						pins::WashValve::Set();
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
						else if (sensors::Valve.value()) {
							pump.atomic_time = false;
							memory();
							pump.reset_timer();
						}
					}
				}

				/*if (state != States::POST_FILL_WASH && Tank::state() == Tank::States::Full) {
					pins::WashValve::Set();
					state = States::POST_FILL_WASH;
				}
				else if (state == States::POST_FILL_WASH && timer((const unsigned char)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
					state = States::DONE;
					return false;
				}
				else if (wLack()) {
					return false;
				}
				else {
					if (pump.state() == pump.OverPumping) {
						exit(EXIT_SUCCESS);
						return false;
					}
					else if (sensors::Valve.value()) {
						pump.atomic_time = false;
						memory();
						pump.reset_timer();
					}
				}*/
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
				pins::WashValve::Set();
			}

			~PostFillWash(void) {
				pump.off();
			}

			operator bool(void) override {
				check_btn_state();

				if (wLack() || timer((const unsigned char)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
					state = State::POST_FILL;
					return false;
				}
				return true;
			}
	};

	class PostFill : public Task {
		protected:
			State virtual inline constexpr task(void) override {
				return State::POST_FILL;
			}

		public:
			//PostFill(void) {
			//	/*pins::Valve::Set();*/
			//	pins::WashValve::Clear();
			//}

			~PostFill(void) {
				pins::Valve::Clear();
				memory();
				pins::WashValve::Clear();

				if (!interrupted) {
					sensors::Valve.stats_new_iteration();
				}

				state = State::STANDBY;
				timer((const unsigned char)delaysID::WASH_TIMER, delay::WASH_TIMER);
			}

			operator bool(void) override {
				//check_btn_state();

				return !timer((const unsigned char)delaysID::POST_FILL, delay::POST_FILLING);
			}
		};
}

template<typename Port, unsigned char mask = 0xff>
void inline hizPort(void) {
	memory();
	Port::SetConfiguration(mask, Port::In);
	memory();
	Port::SetPullUp(mask, Port::PullUp);
	memory();
}

void __attribute__((used, naked, section(".init1")))
pre_init(void) {
	MCUCR = 0x00;

	pins::Valve::SetConfiguration(pins::Valve::Port::Out);
	pins::Valve::Clear();

	pins::WashValve::SetConfiguration(pins::WashValve::Port::Out);
	pins::WashValve::Clear();
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
	WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1); //bit(WDP2);

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
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void __attribute__((used, naked, section(".fini0")))
terminate(void) {
	cli();
	pump.off();
	pins::Valve::Clear();
	pins::WashValve::Clear();
	memory();
	sleep_mode();
}

__attribute__((OS_main))
int main(void) {
	for (;;) {
		#ifdef DEBUG
			bitWrite(PORTB, PINB0, Tank::state() == Tank::Empty);
			bitWrite(PORTD, PIND5, sensors::High.value() && sensors::Low.value());
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

			case State::POST_FILL:
				task::PostFill()();
				break;
		}
	}
}

ISRf(INT0_vect) {
	sensors::High.interruptCallback();
}

ISRf(INT1_vect) {
	sensors::WaterLack.interruptCallback();
}

ISRf(INT2_vect) {
	sensors::Valve.interruptCallback();
	memory();
	pump.atomic_time = false;
}

ISRf(INT3_vect) {
	sensors::Low.interruptCallback();
}

ISRf(PCINT0_vect) {
	buttons::Btn.interruptCallback();
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