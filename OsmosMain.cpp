/*
 * OsmosFilterV3.cpp
 *
 * Created: 20.04.2019 23:35:50
 * Author : Maks
 */

#define F_CPU 16'000'000UL

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
//#include "uart.h"
#include "delay.h"

//#define DEBUG 1

#define BUTTON
#define BUZZER
#define PUMP_SOFT_START

#include "delays.h"
#include "Pins.h"
#include "Pump.h"
#include "reg.h"
//#include "sPWM.h"
//#include "PWM.h"
#include "Buzzer.h"

 FUSES = {
 	.low = FUSE_SUT1, // 0xDF,  FUSE_CKSEL0
 	.high = FUSE_BOOTRST & FUSE_SPIEN & FUSE_BOOTSZ1 & FUSE_BOOTSZ0, // 0xD8,
 	.extended = FUSE_BODLEVEL2 & FUSE_BODLEVEL1 & FUSE_BODLEVEL0 // 0xC8
 };

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

#ifdef BUZZER
using Buzz = Buzzer<pin::Buzzer>;
Buzz buzzer;

const Buzz::Note Done[]{ {Buzz::SI<8>::top(), 225u}, {Buzz::SOL<8>::top(), 225u}, Buzz::nullNote };
const Buzz::Note InvDone[]{ {Buzz::SOL<8>::top(), 225u}, {Buzz::SI<8>::top(), 225u}, Buzz::nullNote };
const Buzz::Note Alarm[]{
	{Buzz::SOL<8>::top(), 75u}, {Buzz::SI<8>::top(), 50u},
	{Buzz::SOL<8>::top(), 75u}, {Buzz::SI<8>::top(), 50u},
	{Buzz::SOL<8>::top(), 75u}, {Buzz::SI<8>::top(), 50u},
	{Buzz::SOL<8>::top(), 75u}, {Buzz::SI<8>::top(), 50u},
	Buzz::nullNote };
#endif

namespace sensor {
	namespace classes {
		using namespace pin;

		using High = Sensor<HighSensor, uint8_t, delay::threshold::high::on, delay::threshold::high::off>;
		using Low = Sensor<LowSensor, uint8_t, delay::threshold::low::on, delay::threshold::low::off>;

		class Valve : public Sensor<ValveOpened, uint8_t, delay::threshold::valve::on, delay::threshold::valve::off> {
			public:
				class Stats {
					public:
						bool freeze = true;

					private:
						friend Valve;

						MedianFilteredValue <uint16_t, 16u> current;
						uint16_t staging;

						uint16_t inline get_staging(void) {
							volatile uint16_t tmp;
							memory();
							ATOMIC{
								tmp = staging;
								memory();
								staging = 0u;
							}
							memory();
							return tmp;
						}

						void inline __attribute__((always_inline)) timerCallback(void) {
							if (!freeze) {
								overflow_limit_sum(staging, 1u);
							}
						}

					public:
						inline __attribute__((always_inline)) Stats(void) : staging(0u) {}

						uint16_t inline __attribute__((always_inline)) operator()(void) {
							return current();
						}
				} stats;

			private:
				typedef Sensor<ValveOpened, uint8_t, delay::threshold::valve::on, delay::threshold::valve::off> base;

			public:
				//Valve(void) : base() {}

				void inline __attribute__((always_inline)) operator()(void) override {
					base::operator()();
					memory();
					if (value()) {
						stats.timerCallback();
					}
				}

				inline operator bool(void) {
					const uint16_t tmp = stats.operator()();
					volatile uint16_t tmp_stg;
					memory();
					if (tmp > 0u) {
						ATOMIC{
							tmp_stg = stats.staging;
						}
						memory();
						return tmp <= tmp_stg;
					}
					return false;
				}

				void inline __attribute__((always_inline)) stats_new_iteration() {
					stats.current(stats.get_staging());
				}
		};

		class WaterLack : public Sensor<pin::WaterLack, uint16_t, delay::threshold::lack::on, delay::threshold::lack::off> {
			protected:
				typedef Sensor<pin::WaterLack, uint16_t, delay::threshold::lack::on, delay::threshold::lack::off> base;
				friend ISRf(INT3_vect);

				enum class WaterLackMode : const uint8_t {
					Normal,
					Lost,
					Waiting,
				};

				uint16_t lost_timer;

				WaterLackMode mode;

				uint16_t get_timer_delay(const bool state) override {
					if (mode == WaterLackMode::Waiting) {
						return delay::threshold::lack::waiting_water;
					}
					return base::get_timer_delay(state);
				}

				void inline __attribute__((always_inline)) handler_timer_done(void) override {
					if (stage_val != value()) {
						if (stage_val) {
							mode = WaterLackMode::Normal;
							#ifdef BUZZER
								memory();
								buzzer.tone(InvDone);
							#endif
						}
						else {
							mode = WaterLackMode::Lost;
							this->time = delay::threshold::lack::without_water;
							memory();
							#ifdef BUZZER
								buzzer.tone(Done);
								memory();
							#endif
						}
						base::handler_timer_done();
					}
					else if (!value() && mode == WaterLackMode::Lost) {
						mode = WaterLackMode::Waiting;
					}
				}

			public:
				inline __attribute__((always_inline)) WaterLack(void) : base(), mode(curr_val ? WaterLackMode::Normal : WaterLackMode::Lost) {};

		};
	}

	classes::High High;
	classes::Low Low;
	classes::Valve Valve;
	classes::WaterLack WaterLack;
}

#include "Tank.h"

#ifdef PUMP_SOFT_START
PumpSoft<pin::Pump> pump;
#else
Pump<pin::Pump> pump;
#endif // PUMP_SOFT_START

Timer<uint16_t> timer(0xffu);

#ifdef BUTTON
namespace button {
	namespace reg {
		namespace btn {
			using msk = flag::pcmsk0::pcint0;
			using flg = flag::pcifr::pcif0;
			//using timer_flg = flag::wdtcsr::wdif;
		}
	}

	namespace classes {
		using Btn = ButtonAssync<pin::Btn, uint8_t, reg::btn::msk, reg::btn::flg>;
	}

	classes::Btn Btn;
}
#endif // BUTTON

namespace wd {
#ifdef BUTTON
	callback* const callbacks[]{ &sensor::High, &sensor::Low, &sensor::Valve, &sensor::WaterLack, &pump, &timer, &button::Btn };
#else
	callback* const callbacks[]{ &sensor::High, &sensor::Low, &sensor::Valve, &sensor::WaterLack, &pump, &timer };
#endif
	constexpr uint8_t const count = (sizeof(callbacks) / sizeof(callbacks[0])) - 1u;
	uint8_t counter = 0u;
}

enum class State : const uint8_t {
	STANDBY,

	PRE_WASH,
	WASH,
	POST_WASH,

	PRE_FILL,
	FILL,

	POST_FILL_WASH,
	DONE,

	FINISHING,

	AUTO_WASH
};

static State state = State::STANDBY;

enum class delaysID : const uint8_t {
	PRE_WASH,
	WASH,
	POST_WASH,
	PRE_FILL,
	POST_FILL_WASH,
	FINISHING,
	FAST_WASH_PERIOD,
};

//Soft_uart_tx<pin::uart_tx> uart;

void inline __attribute__((always_inline)) conf_wtdg(void) {
	WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1);
}

void inline __attribute__((always_inline)) clr_wtdg(void) {
	//bitSet(WDTCSR, WDIE);
	conf_wtdg();
}

namespace task {
	class Task {
		private:
			#ifdef BUZZER
			static uint8_t tone_id;

			static void tone_atom_id(uint16_t fr, uint8_t duration, uint8_t i = 0u) {
				if (tone_id != i) {
					buzzer.tone_atom(fr, duration);
					tone_id = i;
				}
			}

			static void inline tone_atom_id(const Buzz::Note * notelist, uint8_t i = 0u) {
				if (tone_id != i) {
					buzzer(notelist);
					tone_id = i;
				}
			}
			#endif		

		protected:
			// Dont change order/position, starting from ON_HOLD
			enum class BtnState : const uint8_t {
				FREE,
				ON_HOLD,
				CLICK,
				PRESS,
				LONG_PRESS,
				LONG_LONG_PRESS,
			};

			State virtual inline constexpr task(void) = 0;

			static inline bool interrupted = false;

			#ifdef BUTTON
			static inline bool ignore_lack_sensor = false;

			static BtnState btn_state(void) {
				const auto time = button::Btn.timer();
				memory();
				if (!button::Btn) {
					const auto clicks_count = button::Btn.clicks();

					memory();
					if (time || clicks_count) {
						button::Btn.reset();
					}
					memory();

					if (time > delay::BTN_LONG_LONG_TIMER) {
						return BtnState::LONG_LONG_PRESS;
					}
					else if (time > delay::BTN_LONG_TIMER) {
						return BtnState::LONG_PRESS;
					}
					else if (time > delay::BTN_TIMER) {
						return BtnState::PRESS;
					}
					else if (clicks_count) {
						return BtnState::CLICK;
					}
				}
				else {
					#ifdef BUZZER
					if (time > delay::BTN_LONG_LONG_TIMER) {
						tone_atom_id(Buzz::RE2<8>::top(), 200u, 3u);
					}
					else if (time > delay::BTN_LONG_TIMER) {
						tone_atom_id(Buzz::MI<8>::top(), 200u, 2u);
					}
					else if (time > delay::BTN_TIMER) {
						tone_atom_id(Buzz::FA<8>::top(), 200u, 1u);
					}
					#endif
					return BtnState::ON_HOLD;
				}
				return BtnState::FREE;
			}

			void static inline check_btn_state(void) {
				if (ignore_lack_sensor && BtnState::CLICK == btn_state()) {
					ignore_lack_sensor = false;
					button::Btn.disable();
				}
			}
			#endif // BUTTON

			
			static bool inline wLack_body(bool wl) {
				//const bool wl = !sensor::WaterLack.value();
				//memory();
				if (wl) {
					interrupted = true;
				#ifdef BUTTON
					if (!ignore_lack_sensor) {
						state = State::FINISHING;
					}
				}
				return wl && !ignore_lack_sensor;
				#else
				}
				return wl;
				#endif // BUTTON
			}

			static bool inline __attribute__((always_inline)) wLack(void) {
				const bool wl = !sensor::WaterLack.value();
				memory();
				return wLack_body(wl);
			}

		public:
			virtual operator bool(void) = 0;

			void operator()(void) {
				for (; *this;) {
					if (task() == State::STANDBY) {
						memory();
						ATOMIC{
							if (!bitRead(PLLCSR, PLOCK) && bitmask_Read_All(PRR0, bit(PRTIM1) | bit(PRTIM0)) && bitRead(PRR1, PRTIM3)) {
								set_sleep_mode(SLEEP_MODE_STANDBY);
							}
							else {
								set_sleep_mode(SLEEP_MODE_IDLE);
							}
						}
						memory();
					}
					else {
						set_sleep_mode(SLEEP_MODE_IDLE);
					}
					memory();
					sleep_mode();
					memory();
					ATOMIC{
						clr_wtdg();
					}
				}
			}
	};
	#ifdef BUZZER
	uint8_t Task::tone_id = 0u;
	#endif

	class WashTask : public Task {
		public:
			enum class State : const uint8_t {
				//UNKNOWN,
				SKIP,

				AUTO,
				FAST,
				FULL
			};

			static inline State state = State::FULL;

			static void inline __attribute__((always_inline)) increment_skipped_autowashes(void) {
				overflow_limit_sum(skipped_autowashes, (uint8_t)1u, (uint8_t)0u, (uint8_t)simple_round((double)delay::AUTO_WASH_LIMIT / delay::WASH));
				/*if (skipped_autowashes < (const uint8_t)simple_round((double)delay::AUTO_WASH_LIMIT / delay::WASH)) {
					skipped_autowashes++;
				}*/
			}

			static bool inline __attribute__((always_inline)) is_skipped_autowashes(void) {
				return skipped_autowashes > 1u;
			}

		protected:
			static inline uint8_t skipped_autowashes = 1u;

			#ifdef BUTTON
			static void inline check_btn_state(void) {
				if (state != State::AUTO) {
					Task::check_btn_state();
				}
			}
			#endif // BUTTON

			static bool wLack(void) {
				bool tmp = !sensor::WaterLack.value();
				memory();
				if (state == State::AUTO) {
					if (tmp) {
						::state = ::State::FINISHING;
					}
					return tmp;
				}
				else {
					return Task::wLack_body(tmp);
				}
			}
	};

	class Stanby : public Task {
		private:
			uint8_t auto_wash_period_encounter;
			#ifdef BUTTON
			bool pwr_dwn;
			#endif // BUTTON


		protected:
			State virtual inline constexpr task(void) override {
				return State::STANDBY;
			}

		public:
			Stanby(void) : auto_wash_period_encounter(0u) {
				//uart((unsigned short int)0xCCCCu);
				/*pump.off();
				memory();*/
				
				// Already should be closed, but just in case
				// TODO: Check also just in case pump state and maybe terminate program at all
				pin::Valve::Clear();
				memory();
				pin::WashValve::Clear();
				memory();

				#ifdef BUTTON
				pwr_dwn = false;
				ignore_lack_sensor = false;
				button::Btn.reset();
				memory();
				button::Btn.enable();
				#endif // BUTTON
				WashTask::state = WashTask::State::SKIP;
			}

			~Stanby(void) {
				//uart((unsigned short int)0xDDDDu);
				sensor::Valve.stats.freeze = true;
				state = WashTask::state == WashTask::State::SKIP ? State::PRE_FILL : State::PRE_WASH;

				#ifdef BUTTON
				if (ignore_lack_sensor) {
					button::Btn.reset();
				}
				else {
					button::Btn.disable();
				}
				#endif // BUTTON
			}

			operator bool(void) override {
				const Tank::State tmp = Tank::state();
				const bool lck = sensor::WaterLack.value();
				#ifdef BUTTON
				memory();
				BtnState btn_test = btn_state();
				memory();
				// Switch the 'power' state
				if (btn_test == BtnState::LONG_LONG_PRESS) {
					buzzer(Alarm);
					pwr_dwn = !pwr_dwn;
				}
				else {
					if (pwr_dwn) {
						// Just play if somekeys was pressed
						if (btn_test > BtnState::ON_HOLD) {
							buzzer(Alarm);
						}
					}
					// If on power and tank not full process other button events
					else if (tmp != Tank::State::Full) {
						switch (btn_test) {
						case BtnState::LONG_PRESS:
							ignore_lack_sensor = true;
						case BtnState::PRESS:
							if (!lck && !sensor::WaterLack.val_in_sync()) {
								sensor::WaterLack.timer(0u);
							}
						case BtnState::CLICK:
							if (tmp != Tank::State::Empty) {
								interrupted = true;
							}
							pump.state(pump.Ready);
						}
					}
				}
				#endif // BUTTON
				
				bool auto_wash = WashTask::is_skipped_autowashes();
				memory();
				if (timer.isDone()) {
					if (auto_wash_period_encounter < (const uint8_t) simple_round((double)delay::AUTO_WASH_PERIOD / delay::FAST_WASH_PERIOD)) {
						WashTask::state = (WashTask::state == WashTask::State::SKIP) ? WashTask::State::FAST : WashTask::State::FULL;
						auto_wash_period_encounter++;
						timer.timer(delay::FAST_WASH_PERIOD);
					}
					else {
						auto_wash = true;
						if (!lck) {
							WashTask::increment_skipped_autowashes();
							auto_wash_period_encounter = 0u;
							timer.timer(delay::FAST_WASH_PERIOD);
						}
					}
				}

				/*memory();
				static unsigned short int uart_prev = 0;
				unsigned short int uart_buf = (unsigned short int)((static_cast<unsigned short int>(tmp) << 14) | ((lck ? 0x01u : 0x00u) << 13) | ((button::reg::btn::msk::isSet() ? 0x01u : 0x00u) << 12) | ((pin::Btn::IsSet() ? 0x01u : 0x00u) << 11) | (button::Btn.timer() << 4) | static_cast<unsigned char>(btn_test));
				if (uart_buf != uart_prev) {
					uart(uart_buf);
					uart_prev = uart_buf;
				}*/
				memory();


				#ifdef BUTTON
				// Wait till btn is up
				if (!pwr_dwn && btn_test != BtnState::ON_HOLD) {
					if ((tmp == Tank::State::Empty ||
						(tmp != Tank::State::Full && interrupted)) &&
						(lck || ignore_lack_sensor) && pump.state() == pump.Ready) {
						return false;
					}
					else if (auto_wash && lck) {
						WashTask::state = WashTask::State::AUTO;
						return false;
					}
				}
				#else
				if ((tmp == Tank::State::Empty ||
					(tmp != Tank::State::Full && interrupted)) &&
					lck && pump.state() == pump.Ready) {
					return false;
				}
				else if (auto_wash && lck) {
					WashTask::state = WashTask::State::AUTO;
					return false;
				}
				#endif // BUTTON
				return true;
			}
	};

	/*class PreWash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::PRE_WASH;
			}

		public:
			PreWash(void) {
				pin::WashValve::Set();
				memory();
				pin::Valve::Set();
			}

			operator bool(void) override {
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON

				if (wLack()) {
					return false;
				}
				else if (timer((uint8_t)delaysID::PRE_WASH, delay::PRE_WASH)) {
					::state = ::State::WASH;
					return false;
				}

				return true;
			}
	};*/

	class Wash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::WASH;
			}

		public:
			Wash(void) {
				pin::WashValve::Set();
				memory();
				pin::Valve::Set();
			}

			~Wash(void) {
				pin::WashValve::Clear();
			}

			operator bool(void) override {
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON

				if (wLack()) {
					pump.off();
					return false;
				}
				else {
					switch (::state) {
						case ::State::PRE_WASH:
							if (timer((uint8_t)delaysID::PRE_WASH, delay::PRE_WASH)) {
								::state = ::State::WASH;
								pump.force_on();
							}
							break;

						case ::State::WASH:
							if (timer((uint8_t)delaysID::WASH, state == State::FAST ? delay::FAST_WASH : ((delay::WASH - 1u) * skipped_autowashes) + 1u)) {
								::state = ::State::POST_WASH;
								skipped_autowashes = 1u;
								//pin::Pump::Clear();
								pump.force_off();
								//return false;
							}
							break;

						case ::State::POST_WASH:
							if (timer((const uint8_t)delaysID::POST_WASH, delay::POST_WASH)) {
								if (state == State::AUTO) {
									::state = ::State::FINISHING;
								}
								else {
									::state = ::State::PRE_FILL;
								}
								return false;
							}
							break;
					}
				}
				
				return true;
			}
	};

	/*class PostWash : public WashTask {
		protected:
			::State virtual inline constexpr task(void) override {
				return ::State::POST_WASH;
			}

		public:
			~PostWash(void) {
				pin::WashValve::Clear();
			}

			operator bool(void) override {
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON
				if (wLack()) {
					return false;
				}
				else if (timer((const uint8_t)delaysID::POST_WASH, delay::POST_WASH)) {
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
	};*/

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
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON

				if (wLack()) {
					return false;
				}
				else if (timer((const uint8_t)delaysID::PRE_FILL, delay::PRE_FILLING)) {
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
				pin::FlowReg::Set();
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
				pin::FlowReg::Clear();
			}

			operator bool(void) override {
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON

				if (state == State::POST_FILL_WASH) {
					if (timer((const uint8_t)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
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
				#ifdef BUTTON
				check_btn_state();
				memory();
				#endif // BUTTON

				if (wLack() || timer((const uint8_t)delaysID::POST_FILL_WASH, delay::POST_FILL_WASH)) {
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
			Finishing(void) {
				pump.off();
			}

			~Finishing(void) {
				pin::Valve::Clear();
				memory();
				pin::WashValve::Clear();
				memory();

				if (!interrupted) {
					sensor::Valve.stats_new_iteration();
				}

				state = State::STANDBY;
				timer((const uint8_t)delaysID::FAST_WASH_PERIOD, delay::FAST_WASH_PERIOD);
			}

			operator bool(void) override {
				return !timer((const uint8_t)delaysID::FINISHING, delay::FINISHING);
			}
		};
}

template<typename Port, uint8_t mask = 0xffu>
void inline __attribute__((always_inline)) hizPort(void) {
	memory();
	Port::SetConfiguration(mask, Port::In);
	memory();
	Port::SetPullUp(mask, Port::PullUp);
	memory();
}

void __attribute__((used, naked, section(".init1")))
pre_init(void) {
	MCUSR = 0u;
	memory();
	MCUCR = bit(JTD);
	memory();
	MCUCR = bit(JTD);
	memory();

	pin::Valve::SetConfiguration(pin::Valve::Port::Out);
	pin::Valve::Clear();

	pin::WashValve::SetConfiguration(pin::WashValve::Port::Out);
	pin::WashValve::Clear();

	pin::FlowReg::SetConfiguration(pin::FlowReg::Port::Out);
	pin::FlowReg::Clear();

	/*pin::uart_tx::SetConfiguration(pin::uart_tx::Port::Out);
	pin::uart_tx::Clear();*/
	memory();

	#ifdef DEBUG
		bitSet(DDRB, PINB0);
		bitSet(DDRD, PIND5);
	#else
		//hizPort<Mcucpp::IO::Portb, 0xC6>();
		//hizPort<Mcucpp::IO::Portd, 0xF0>();
		//hizPort<Mcucpp::IO::Portc, 0xBF>();
		constexpr uint8_t prtBmsk = 0xFFu & ~(
			bit(pin::Pump::Number) |
			bit(pin::WashValve::Number) | 
			bit(pin::Btn::Number) | 
			bit(pin::Buzzer::Number));
		hizPort<Mcucpp::IO::Portb, prtBmsk>();
		constexpr uint8_t prtDmsk = 0xFFu & ~(
			bit(pin::HighSensor::Number) | 
			bit(pin::WaterLack::Number) | 
			bit(pin::ValveOpened::Number) | 
			bit(pin::LowSensor::Number) |
			bit(pin::Valve::Number) | 
			bit(pin::FlowReg::Number));
		hizPort<Mcucpp::IO::Portd, prtDmsk>();
		hizPort<Mcucpp::IO::Portc>();
		hizPort<Mcucpp::IO::Porte>();
		hizPort<Mcucpp::IO::Portf>();
	#endif // DEBUG
	
	memory();
	bitClear(ADCSRA, ADEN);
	memory();
	bitSet(ACSR, ACD);
	memory();

	//PRR0 = bit(PRTWI) | bit(PRTIM0) | bit(PRTIM1) | bit(PRSPI) | bit(PRADC);
	//PRR1 = bit(PRUSB) | bit(PRTIM4) | bit(PRTIM3) | bit(PRUSART1);
	PRR0 = bit(PRTWI) | bit(PRTIM1) | bit(PRSPI) | bit(PRADC);
	PRR1 = bit(PRUSB) | bit(PRTIM4) | bit(PRTIM3) | bit(PRUSART1);
}

void __attribute__((used, naked, section(".init8")))
init(void) {
	cli();
	memory();
	wdt_reset(); //asm("wdr");
	memory();
	WDTCSR = bit(WDCE) | bit(WDE);
	memory();
	conf_wtdg();
	//WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1); //bit(WDP2);
	memory();

	// Enable pin change interrupt for button
	PCICR = bit(PCIE0);
	memory();
	PCIFR = 0u;
	memory();
	PCMSK0 = bit(PCINT0);
	memory();

	// Enable Interrupt any edge
	EICRA = bit(ISC30) | bit(ISC20) | bit(ISC10) | bit(ISC00);
	memory();
	EIFR = 0u;
	memory();
	EIMSK = bit(INT0) | bit(INT1) | bit(INT2) | bit(INT3);
	
	//set_sleep_mode(SLEEP_MODE_STANDBY);
	//set_sleep_mode(SLEEP_MODE_IDLE);

	/*constexpr unsigned int period = simple_round((double)F_CPU / 8 / 38400);
	uart.init(bit(CS01), period);
	uart((unsigned short int)0b10101010'10101010u);*/
}

void __attribute__((used, naked, section(".fini0")))
terminate(void) {
	//uart((unsigned short int)0xBBBBu);
#ifdef BUZZER
	pump.emergency_off();
	pin::Valve::Clear();
	pin::WashValve::Clear();
	pin::FlowReg::Clear();
	memory();
	buzzer(Alarm);
	memory();
	while (!buzzer.sleep_ready()){}
	memory();
	cli();
#else
	cli();
	pump.emergency_off();
	pin::Valve::Clear();
	pin::WashValve::Clear();
	pin::FlowReg::Clear();
#endif
	memory();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	memory();
	sleep_enable();
	memory();
	sleep_cpu();
}

__attribute__((OS_main))
int main(void) {
	sei();
	memory();
	for (;;) {
		#ifdef DEBUG
			bitWrite(PORTB, PINB0, Tank::state() == Tank::Empty);
			bitWrite(PORTD, PIND5, sensor::High.value() && sensor::Low.value());
		#endif // DEBUG
		
		switch (state) {

			case State::STANDBY:
				task::Stanby()();
				break;

			/*case State::PRE_WASH:
				task::PreWash()();
				break;

			case State::WASH:
				task::Wash()();
				break;

			case State::POST_WASH:
				task::PostWash()();
				break;*/
			case State::PRE_WASH:
			case State::WASH:
			case State::POST_WASH:
				task::Wash()();
				break;

			case State::PRE_FILL:
				task::PreFill()();
				break;

			case State::FILL:
				task::Fill()();
				break;

			case State::FINISHING:
				task::Finishing()();
				break;
		}
			//buzzer(Alarm);
	
			//pump.soft_start.start();
			//Mcucpp::delay_ms<3000, F_CPU>();
			//pump.soft_start.stop();
			//Mcucpp::delay_ms<5000, F_CPU>();
	}
}

ISRf(INT0_vect) {
	clr_wtdg();
	memory();
	sensor::High.interruptCallback();
	memory();
	clr_wtdg();
}

ISRf(INT1_vect) {
	clr_wtdg();
	memory();
	sensor::WaterLack.interruptCallback();
	memory();
	clr_wtdg();
}

ISRf(INT2_vect) {
	clr_wtdg();
	memory();
	sensor::Valve.interruptCallback();
	memory();
	pump.atomic_time = false;
	memory();
	clr_wtdg();
}

ISRf(INT3_vect) {
	clr_wtdg();
	memory();
	sensor::Low.interruptCallback();
	memory();
	clr_wtdg();
}

#ifdef BUTTON
ISRf(PCINT0_vect) {
	clr_wtdg();
	memory();
	button::Btn.interruptCallback();
	memory();
	clr_wtdg();
#ifdef BUZZER
	memory();
	if (button::Btn.operator bool()) {
		buzzer.tone(Buzz::SOL<8>::top(), 50u);
	}
	memory();
	clr_wtdg();
#endif
}
#endif // BUTTON

ISRf(WDT_vect) {
	clr_wtdg();
	memory();
	wd::callbacks[wd::counter]->operator()();
	memory();
	if (wd::counter < wd::count) {
		wd::counter++;
	}
	else {
		wd::counter = 0u;
	}
	memory();
	clr_wtdg();
}

#ifdef PUMP_SOFT_START
ISRf(TIMER0_OVF_vect) {
	clr_wtdg();
	memory();
	pump.timerCallback();
	memory();
	clr_wtdg();
}
#endif

//ISRf(TIMER0_COMPA_vect) {
//	uart.timer_callback();
//}

#ifdef BUZZER
ISRf(TIMER3_COMPA_vect) {
	clr_wtdg();
	memory();
	buzzer.timerCallback();
	memory();
	clr_wtdg();
}
#endif
