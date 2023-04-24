#pragma once

#ifndef BUZZER_H_
#define BUZZER_H_

#include "Common.h"
#include "atomic.h"

#ifdef TIMER1_OVF_vect
ISRv(TIMER1_OVF_vect);
#endif

#ifdef TIMER3_OVF_vect
ISRv(TIMER3_OVF_vect);
#endif

#ifdef TIMER3_COMPA_vect
ISRv(TIMER3_COMPA_vect);
#endif

#ifdef WDT_vect
ISRv(WDT_vect)
#endif

template<class Pin>
class Buzzer {
	private:
		template<double base_note, uint8_t oct>
		class N {
			public:
				static constexpr uint16_t freq = base_note * (1u << (oct - 1u)) + 0.5d;

				static constexpr uint16_t top(void) {
					return conv_freq(freq);
				}
			};
	public:
		struct Note {
			const uint16_t freq;
			const uint8_t time;
		};

		static inline const Note nullNote { 0u, 0u };

		// C
		template<uint8_t oct>
		class DO : public N <32.7d, oct>{};
		// C#
		template<uint8_t oct>
		class DO2 : public N <34.65d, oct> {};
		// D
		template<uint8_t oct>
		class RE : public N <36.95d, oct> {};
		// D#
		template<uint8_t oct>
		class RE2 : public N <38.88d, oct> {};
		// E
		template<uint8_t oct>
		class MI : public N <41.21d, oct> {};
		// F
		template<uint8_t oct>
		class FA : public N <43.65d, oct> {};
		// F#
		template<uint8_t oct>
		class FA2 : public N <46.25d, oct> {};
		// G
		template<uint8_t oct>
		class SOL : public N <49.0d, oct> {};
		// G#
		template<uint8_t oct>
		class SOL2 : public N <51.90d, oct> {};
		// A
		template<uint8_t oct>
		class LA : public N <55.0d, oct> {};
		// A#
		template<uint8_t oct>
		class LA2 : public N <58.26d, oct> {};
		// B
		template<uint8_t oct>
		class SI : public N <61.74d, oct> {};

	private:
		static const uint16_t max_top = UINT_MAX;

		class Timer {
			private:
				typedef uint16_t T;

				void static inline __attribute__((always_inline)) reset(void) {
					TCNT3 = 0u;
				}

				void static inline __attribute__((always_inline)) reset_interrupt(void) {
					bitSet(TIFR3, TOV3);
				}

				void static inline __attribute__((always_inline)) reset_atom(void) {
					ATOMIC{
						reset();
					}
				}

				void static inline __attribute__((always_inline)) reset_interrupt_atom(void) {
					ATOMIC{
						reset_interrupt();
					}
				}

				void static inline __attribute__((always_inline)) top(T val) {
					OCR3A = val;
				}

				T static inline __attribute__((always_inline)) top(void) {
					return OCR3A;
				}

				void static inline __attribute__((always_inline)) top_atom(T val) {
					ATOMIC{
						top(val);
					}
				}

				T static inline __attribute__((always_inline)) top_atom(void) {
					T tmp;
					memory();
					ATOMIC{
						tmp = top();
					}
					memory();
					return tmp;
				}

			protected:
				#ifdef TIMER1_OVF_vect
					friend ISRf(TIMER1_OVF_vect);
				#endif

				#ifdef WDT_vect
					friend ISRf(WDT_vect);
				#endif

			public:
				void static inline __attribute__((always_inline)) init(void) {
					on();
					memory();
					TCCR3A = /*bit(WGM31) | bit(WGM30)*/0u;
					memory();
					//bitSet(TIMSK3, TOIE3);
					TIMSK3 = bit(OCIE3A);
					memory();
					top(UINT_MAX);
					memory();
					TCCR3B = /*bit(WGM33) |*/ bit(WGM32) | bit(CS32) | bit(CS30);
					memory();
					off();
					memory();
					reset_interrupt();
				}

				void static inline __attribute__((always_inline)) init_atom(void) {
					ATOMIC{
						init();
					}
				}

				T static constexpr calc_period(uint8_t ms) {
					//constexpr double k = ((double)F_CPU / 1024u) / 1000u;
					constexpr uint32_t k = F_CPU / 1000u;
					//constexpr uint8_t top = 8u;
					//constexpr uint8_t bot = ((double)top / k) - 0.5d;
					//return ((T)ms * top) / bot;
					return ms ? ((k * ms) >> 10u) - 1u  :  0u;
				}

				void static inline start(void) {
					bitmask_Set(TCCR3B, bit(CS32) | bit(CS30));
				}

				void static inline stop(void) {
					bitmask_Clear(TCCR3B, bit(CS32) | bit(CS30));
				}

				void static inline start_atom(void) {
					ATOMIC{
						start();
					}
				}

				void static inline stop_atom(void) {
					ATOMIC{
						stop();
					}
				}

				void static inline on(void) {
					bitClear(PRR1, PRTIM3);
				}

				void static inline off(void) {
					bitSet(PRR1, PRTIM3);
				}

				void static inline on_atom(void) {
					ATOMIC {
						on();
					}
				}

				void static inline off_atom(void) {
					ATOMIC {
						off();
					}
				}

				void static inline timer(T val) {
					if (val == 0) {
						stop();
						memory();
						//top(val);
						memory();
						reset_interrupt();
						memory();
						reset();
						memory();
						off();
					}
					else {
						on();
						memory();
						top(val);
						memory();
						reset_interrupt();
						memory();
						reset();
						/*memory();
						start();*/
					}
				}

				void static inline timer_atom(T val) {
					ATOMIC{
						timer(val);
					}
				}

				void static inline __attribute__((always_inline)) timer_ms(uint8_t val) {
					timer(calc_period(val));
				}

				void static inline __attribute__((always_inline)) timer_ms_atom(uint8_t val) {
					timer_atom(calc_period(val));
				}

				bool static inline is_running(void) {
					return bitRead(PRR1, PRTIM3);
				}
		};

		const Note * volatile notelist;

		const Note EmptyList[1] { nullNote };

		//static const uint8_t note_start_pos = 0u;
		//uint8_t note_position;

		const Note get_note(void) {
			Note tmp = *notelist;
			//memory();
			if (tmp.time != 0u) {
				notelist++;
			}
			return tmp;
		}

		static uint16_t inline __attribute__((always_inline)) top(void) {
			return OCR1A;
		}

		static void inline __attribute__((always_inline)) top(uint16_t val) {
			OCR1A = val;
		}

		static uint16_t inline top_atom(void) {
			uint16_t tmp;
			memory();
			ATOMIC{
				tmp = top();
			}
			memory();
			return tmp;
		}

		static void inline top_atom(uint16_t val) {
			ATOMIC{
				top(val);
			}
		}

		static void freq(uint16_t val) {
			if (val >= max_top) {
				top(max_top);
				/*memory();
				stop();
				memory();
				off();
				memory();
				Pin::Set();*/
			}
			else {
				if (val == 0u) {
					stop();
					memory();
					off();
					memory();
					Pin::Clear();
				}
				else {
					on();
					memory();
					top(val);
					memory();
					start();
				}
			}
		}

		//static uint16_t freq_atom(void);

		static void freq_atom(uint16_t val) {
			ATOMIC{
				freq(val);
			}
		}

	protected:
		#ifdef TIMER1_OVF_vect
			friend ISRf(TIMER1_OVF_vect);
		#endif

		#ifdef TIMER3_OVF_vect
			friend ISRf(TIMER3_OVF_vect);
		#endif

		#ifdef TIMER3_COMPA_vect
			friend ISRf(TIMER3_COMPA_vect);
		#endif

		#ifdef WDT_vect
			friend ISRf(WDT_vect);
		#endif

		void inline __attribute__((used, always_inline)) timerCallback(void) {
			tone(get_note());
		}

	public:
		void static inline __attribute__((always_inline)) init(void) {
			ATOMIC{
				on();
				memory();
				//TCCR1A = (TCCR1A & ~(bit(COM1A1))) | bit(COM1A0) | bit(WGM11) | bit(WGM10);
				stop(); /*| bit(WGM11) | bit(WGM10)*/;
				memory();
				//TCCR1B = (TCCR1B & ~(bit(CS12) | bit(CS11))) | bit(WGM13) | bit(WGM12) | bit(CS10);
				TCCR1B = /*bit(WGM13) |*/ bit(WGM12) | bit(CS10);
				memory();
				off();
				memory();
				Pin::SetConfiguration(Pin::Port::Out);
				Pin::Clear();
				memory();
				Timer::init();
			}
		}

		static constexpr uint16_t conv_freq(uint16_t freq) {
			constexpr double tmp = (double)F_CPU / 2u;
			return freq ? (tmp / freq) - 0.5d : 0u;
		}

		inline __attribute__((always_inline)) Buzzer(void): /*note_position(note_start_pos),*/ notelist(EmptyList) {
			init();
		}

		void static inline start(void) {
			TCCR1A = bit(COM1A0);
		}

		void static inline start_atom(void) {
			ATOMIC{
				start();
			}
		}

		void static inline stop(void) {
			TCCR1A = 0u;
		}

		void static inline stop_atomic(void) {
			ATOMIC{
				stop();
			}
		}

		void static inline on(void) {
			bitClear(PRR0, PRTIM1);
		}

		void static inline off(void) {
			bitSet(PRR0, PRTIM1);
		}

		void static on_atom(void) {
			ATOMIC{
				on();
			}
		}

		void static off_atom(void) {
			ATOMIC{
				off();
			}
		}

		void static inline __attribute__((always_inline)) stop_tone(void) {
			freq_atom(0u);
		}

		void static inline __attribute__((always_inline)) tone(uint16_t fr) {
			//freq(calc_top(fr));
			freq(fr);
		}

		void static inline __attribute__((always_inline)) tone(uint16_t fr, uint8_t duration) {
			//const auto fr_tmp = calc_top(fr);
			//const auto dur_tmp = Timer::calc_period(duration);
			//memory();
			Timer::timer(Timer::calc_period(duration));
			memory();
			freq(fr);
			memory();
			Timer::start();
		}

		void static inline __attribute__((always_inline)) tone(const Note note) {
			tone(note.freq, note.time);
		}

		void inline tone(const Note* list) {
			notelist = list;
			//note_position = note_start_pos;
			memory();
			Timer::timer(Timer::calc_period(1u));
			memory();
			Timer::start();
		}

		void static inline __attribute__((always_inline)) tone_atom(uint16_t fr) {
			//freq_atom(calc_top(fr));
			freq_atom(fr);
		}

		void static inline tone_atom(uint16_t fr, uint8_t duration) {
			/*const auto fr_tmp = calc_top(fr);
			const auto dur_tmp = Timer::calc_period(duration);
			memory();*/
			Timer::timer_atom(Timer::calc_period(duration));
			memory();
			freq_atom(fr);
			memory();
			Timer::start_atom();
		}

		void static inline __attribute__((always_inline)) tone_atom(const Note note) {
			tone(note.freq, note.time);
		}


		void inline __attribute__((always_inline)) operator()(uint16_t freq, uint8_t duration) {
			//notelist = EmptyList;
			//note_position = note_start_pos;
			tone_atom(freq, duration);
		}

		void operator()(const Note *list) {
			ATOMIC{
				tone(list);
			}
		}

		bool static inline sleep_ready(void) {
			return bitRead(PRR0, PRTIM1) && Timer::is_running();
		}
};

#endif