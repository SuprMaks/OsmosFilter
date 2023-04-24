#pragma once

#ifndef PUMP_H_
#define PUMP_H_

#include "atomic.h"

#ifdef TIMER0_OVF_vect
	ISRv(TIMER0_OVF_vect);
#endif

namespace delay {
	namespace threshold {
		namespace lack {
			extern const uint16_t try_water;
			extern const uint16_t try_water_max;
		}
	}
	extern const uint16_t MAX_PUMPING;
}

template<class Pin>
class Pump : public Timer<uint16_t> {
	private:
		typedef uint16_t TimerDT;
		typedef Timer<TimerDT> base;
		TimerDT timer_value;

	protected:
		bool virtual inline raw_state(void) volatile {
			return Pin::IsSet();
		}

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

		friend ISRf(TIMER0_COMPA_vect);

		MedianFilteredValue <TimerDT, 8u> stat;

		TimerDT _incr_delay;

		TimerDT inline __attribute__((always_inline)) incr_delay(void) {
			if (this->_state == State::Cooldown || this->_state == State::Ready) {
				volatile TimerDT tmp;
				memory();
				ATOMIC{
					tmp = _incr_delay;
				}
				memory();
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
							_incr_delay /= 2u;
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
				//tmp = (TimerDT)simple_round((double)stat() * 1.3);

				// a / b == a * (1 / b) == (a * x) >> n
				// x = (2^n) / b + 1
				constexpr uint8_t k = 3u;
				// x: 2^n < (MAX / a_max - 1) * b
				// https://alexgyver.ru/lessons/code-optimisation/
				//constexpr uint32_t n2 = (ULONG_MAX / (delay::MAX_PUMPING + 3000u) - 1u) * k;
				constexpr uint8_t n = 17u;
				constexpr uint16_t x = (1ul << n) / k + 1u;

				tmp = stat() + (((uint32_t)stat() * x) >> n);
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

		inline __attribute__((always_inline)) Pump(void) : base(), _state(State::Ready), _incr_delay(delay::threshold::lack::try_water), atomic_time(true), atomic(true) {
			Pin::SetConfiguration(Pin::Port::Out);
			Pin::Clear();
		}

		void virtual inline __attribute__((always_inline)) force_on(void) {
			Pin::Set();
		}

		void virtual inline __attribute__((always_inline)) force_off(void) {
			Pin::Clear();
		}

		void on(void) {
			if (!raw_state()) {
				force_on();
				memory();
				state(State::Working);
				memory();
				this->reset_timer();
			}
		}

		void off(void) {
			if (raw_state()) {
				force_off();
				memory();
				state(State::Cooldown);
				memory();

				if (atomic) {
					TimerDT sum = 0u;
					while (incr_delay() > delay::threshold::lack::try_water && elapsed_time() > sum) {
						sum += incr_delay();
						ATOMIC{
							_incr_delay /= 2u;
						}
					}
					this->timer(incr_delay());
				}
				else {
					this->timer(incr_delay());
					if (incr_delay() < delay::threshold::lack::try_water_max) {
						ATOMIC{
							_incr_delay *= 2u;
						}
					}
				}
			}
		}

		void virtual inline __attribute__((always_inline)) emergency_off(void) {
			off();
		}
};

template<class Pin>
class PumpSoft : public Pump<Pin> {
	private:
		typedef Pump<Pin> base;

		static constexpr double volt = 24.0d;
		static constexpr double volt_per_step = 0.03d;

		static constexpr auto Tf = 64'000'000ul;
		static constexpr auto f = 100'000u;
		static constexpr uint16_t pwm_steps = (uint16_t)(((double)Tf / f) + 0.5d) - 1u; //(uint16_t)((volt / volt_per_step) + 0.5d) - 1u ;
		static const int8_t start_k = 1, stop_k = -1;

		static constexpr double full_start_ramp_duration = 2.d; //2 seconds
		static constexpr double full_stop_ramp_duration = 1.5d; //1 seconds
		static constexpr double fast_ramp_duration = 0.2d; //10% from full ramp duration

		static constexpr uint16_t fast_pwm_trh = simple_round((double)pwm_steps * ((double)(25u - 6u) / 100u));//25% - 6% driver offset pow(0.2) = 4% torque

		static constexpr double timer_clk = (double)F_CPU / 1024u;
		static constexpr uint16_t slow_ramp_steps = pwm_steps - fast_pwm_trh;

		static constexpr uint8_t start_period = simple_round(((timer_clk / slow_ramp_steps) - 1u) * (1.0d - fast_ramp_duration) * full_start_ramp_duration);
		static constexpr uint8_t fast_start_period = simple_round(((timer_clk / fast_pwm_trh) - 1u) * fast_ramp_duration * full_start_ramp_duration);

		static constexpr uint8_t stop_period = simple_round(((timer_clk / slow_ramp_steps) - 1u) * (1.0d - fast_ramp_duration) * full_stop_ramp_duration);
		static constexpr uint8_t fast_stop_period = simple_round(((timer_clk / fast_pwm_trh) - 1u) * fast_ramp_duration * full_stop_ramp_duration);

		template<uint8_t div>
		class Timer {
			public:
				static void inline __attribute__((always_inline)) period(uint8_t baud_div) {
					OCR0A = baud_div;
				}

				static void inline __attribute__((always_inline)) on(void) {
					bitClear(PRR0, PRTIM0);
				}

				static void inline on_atom(void) {
					ATOMIC{
						on();
					}
				}

				static void inline __attribute__((always_inline)) off(void) {
					bitSet(PRR0, PRTIM0);
				}

				static void inline off_atom(void) {
					ATOMIC{
						off();
					}
				}

				static void inline __attribute__((always_inline)) start(void) {
					TCCR0B = bit(WGM02) | (div & 0b111u);
				}

				/*static void inline start_atom(void) {
					ATOMIC{
						start();
					}
				}*/

				static void inline __attribute__((always_inline)) stop(void) {
					bitmask_Clear(TCCR0B, 0b111u);
				}

				static void inline stop_atom(void) {
					ATOMIC{
						stop();
					}
				}

				static void inline __attribute__((always_inline)) init(void) {
					on();
					memory();
					//bitmask_Set(TCCR0A, bit(WGM01) | bit(WGM00));
					TCCR0A = bit(WGM01) | bit(WGM00);
					bitSet(TIMSK0, TOIE0);
					//bitmask_Set(TCCR0B, div);
					TCCR0B = bit(WGM02);
					reset_interrupt();
					reset();
					memory();
					off();
				}

				static void inline __attribute__((always_inline)) init_atom(void) {
					ATOMIC{
						init();
					}
				}

				// BAUD_DIV = (CPU_CLOCK / DIV) / BAUD_RATE
				static void inline __attribute__((always_inline)) init_atom(const uint8_t baud_div) {
					init_atom();
					period(baud_div);
				}

				void static inline __attribute__((always_inline)) reset(void) {
					TCNT0 = 0u;
				}

				void static inline __attribute__((always_inline)) reset_atom(void) {
					ATOMIC{
						reset();
					}
				}

				void static inline reset_interrupt(void) {
					bitSet(TIFR0, TOV0);
				}

				void static inline reset_interrupt_atom(void) {
					ATOMIC{
						reset_atom();
					}
				}

				bool static inline is_running(void) {
					return bitRead(PRR0, PRTIM0);
				}
		};

		typedef Timer<bit(CS02) | bit(CS00)> timer;

		uint16_t pwm_tch;
		int8_t k;

		void compr(uint16_t val) volatile {
			
			//val <<= 1u;
			memory();
			TC4H = highByte(val);
			memory();
			OCR4B = val;
			memory();
			pwm_tch = val;
			memory();

			/*memory();
			val = val << 1u;
			uint8_t tmp = val >> 8u;
			TC4H = pwm_tch = val >> 8u;
			memory();
			OCR4B = val;
			memory();*/
		}

		uint16_t inline __attribute__((always_inline)) compr(void) volatile {
			/*uint16_t tmp = pwm_tch;
			memory();
			tmp <<= 7u;
			tmp |= (OCR4B >> 1u);
			return tmp;*/
			return pwm_tch;
		}

		void compr_atom(uint16_t val) {
			ATOMIC{
				compr(val);
			}
		}

		uint16_t compr_atom(void) {
			uint16_t tmp;
			memory();
			ATOMIC{
				tmp = compr();
			}
			memory();
			return tmp;
		}

		static inline __attribute__((always_inline)) bool pll_state(void) {
			return bitRead(PLLCSR, PLOCK);
		}

		static inline __attribute__((always_inline)) void wait_start_pll(void) {
			while (!pll_state());
		}

		static inline __attribute__((always_inline)) void start_pll(void) {
			bitSet(PLLCSR, PLLE);
			memory();
			wait_start_pll();
		}

		static inline __attribute__((always_inline)) void start_pll_atom(void) {
			ATOMIC{
				bitSet(PLLCSR, PLLE);
			}
			memory();
			wait_start_pll();
		}

		static inline __attribute__((always_inline)) void start_timer(void) {
			bitClear(PRR1, PRTIM4);
		}

		static inline __attribute__((always_inline)) void start_timer_atom(void) {
			ATOMIC{
				start_timer();
			}
		}

		static inline void _on(void) {
			start_pll();
			memory();
			start_timer();
		}

		static inline void _on_atom(void) {
			start_pll_atom();
			memory();
			start_timer_atom();
		}

		static inline void _off(void) {
			bitSet(PRR1, PRTIM4);
			memory();
			bitClear(PLLCSR, PLLE);
		}

		static inline void _off_atom(void) {
			ATOMIC{
				_off();
			}
		}

		static inline __attribute__((always_inline)) void start(void) {
			TCCR4B = bit(CS40);
		}

		static inline __attribute__((always_inline)) void stop(void) {
			TCCR4B = 0u;
		}

		/*static inline void start_atom(void) {
			ATOMIC{
				start();
			}
		}

		static inline void stop_atom(void) {
			ATOMIC{
				stop();
			}
		}*/

		static void inline top(uint16_t max) {
			TC4H = highByte(max);
			memory();
			OCR4C = max;
			memory();
		}

		static void inline __attribute__((always_inline)) top_atom(uint16_t max) {
			ATOMIC{
				top(max);
			}
		}

		uint16_t inline __attribute__((always_inline)) incr_pwm(int8_t val) {
			uint16_t tmp_pwm = compr();
			memory();
			if (val > 0) {
				if (tmp_pwm >= pwm_steps - val) {
					compr(pwm_steps);
					memory();
					stop();
					memory();
					Pin::Set();
					memory();
					_off();
					memory();
					return pwm_steps;

				}
				else {
					_on();
					memory();
					const auto ret = tmp_pwm + val;
					compr(ret);
					memory();
					//Pin::Clear();
					start();
					memory();
					return ret;
				}
			}
			else {
				if (tmp_pwm <= -val) {
					compr(0u);
					memory();
					stop();
					memory();
					Pin::Clear();
					memory();
					_off();
					memory();
					return 0u;
				}
				else {
					_on();
					memory();
					const auto ret = tmp_pwm + val;
					compr(ret);
					memory();
					//Pin::Clear();
					start();
					memory();
					return ret;
				}
			}
		}

		inline __attribute__((always_inline)) bool pwm_state(void) {
			return compr() > 0u;
		}

		void inline __attribute__((always_inline)) upd_period(uint16_t pwm, bool st) {
			if (st) {
				if (pwm < fast_pwm_trh) {
					timer::period(fast_start_period);
				}
				else {
					timer::period(start_period);
				}
			}
			else {
				if (pwm < fast_pwm_trh) {
					timer::period(fast_stop_period);
				}
				else {
					timer::period(stop_period);
				}
			}
		}

		void inline upd_period_atom(void) {
			if (pwm_state()) {
				if (compr_atom() < fast_pwm_trh) {
					timer::period(fast_start_period);
				}
				else {
					timer::period(start_period);
				}
			}
			else {
				if (compr_atom() < fast_pwm_trh) {
					timer::period(fast_stop_period);
				}
				else {
					timer::period(stop_period);
				}
			}
		}

		void inline start_sequence(void) {
			set_sleep_mode(SLEEP_MODE_IDLE);
			timer::on_atom();
			//2seconds
			//constexpr uint8_t period = simple_round(((double)F_CPU / 1024 / pwm_steps * 2) - 1);
			memory();
			upd_period_atom();
			memory();
			ATOMIC{
				timer::reset_interrupt();
				memory();
				timer::reset();
				memory();
				timer::start();
			}
		}

	protected:
		#ifdef WDT_vect
			friend ISRf(WDT_vect);
		#endif
			friend int main();
		#ifdef TIMER0_COMPA_vect
			friend ISRf(TIMER0_COMPA_vect);
		#endif
		#ifdef TIMER0_OVF_vect
			friend ISRf(TIMER0_OVF_vect);
		#endif

		bool inline __attribute__((always_inline)) raw_state(void) volatile override {
			return k == start_k;
		}

		void inline __attribute__((always_inline)) timerCallback(void) {
			uint16_t pos = incr_pwm(k);
			memory();
			bool st = raw_state();
			upd_period(pos, st);
			memory();
			if ((st && pos == pwm_steps) ||
				(!st && pos == 0u)) {
				timer::stop();
				memory();
				timer::off();
			}
		}

	public:
		inline __attribute__((always_inline)) PumpSoft(void) : base(), k(stop_k), pwm_tch(0u) {
			ATOMIC{
				// Enable 64 MHz PLL and use as source for Timer1
				PLLFRQ = bit(PLLUSB) | bit(PLLTM1) | bit(PDIV3) | bit(PDIV1);
				memory();
				PLLCSR = bit(PINDIV) /*| bit(PLLE)*/;
				/*memory();
				wait_start_pll();*/
				memory();
				start_timer();
				memory();
				TCCR4A = bit(COM4B1) | bit(PWM4B);
				//TCCR4E = bit(ENHC4);
				memory();
				/*TCCR4B = bit(CS40);
				memory();*/
				compr(0u);
				memory();
				top(pwm_steps);
				memory();
				stop();
				//memory();
				//_off();
			}
			memory();
			//Pin::Clear();
			timer::init();
		}

		void inline __attribute__((always_inline)) emergency_off(void) {
			timer::off_atom();
			memory();
			stop();
			memory();
			Pin::Clear();
			_off_atom();
		}

		bool static inline sleep_ready(void) {
			return !pll_state() && timer::is_running();
		}

		void inline force_on(void) override {
			k = start_k;
			memory();
			start_sequence();
		}

		void inline force_off(void) override {
			k = stop_k;
			memory();
			start_sequence();
		}
};

#endif