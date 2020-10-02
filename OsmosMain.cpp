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
#include <avr/wdt.h>
#include "Common.h"
#include "MedianFilter.h"
#include  "math.h"

FUSES = {
	.low = FUSE_CKSEL0, // 0xFE,
	.high = FUSE_BOOTRST & FUSE_SPIEN & FUSE_BOOTSZ1 & FUSE_BOOTSZ0, // 0xD8,
	.extended = FUSE_BODLEVEL2 // 0xFB
};

//#define DEBUG 1

constexpr unsigned char watchdog = 8;
constexpr unsigned char wd_calls = 6;

constexpr auto sensor_tick_rate = 1.0 / ((double)wd_calls / watchdog);

template<typename T>
constexpr auto sec_conv(T val) {
	return (unsigned short int)((val * sensor_tick_rate) + 0.5);
};

namespace delay {
#ifdef DEBUG
#define DEBUG_LEVEL 1
#else
	namespace treshold {
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
			constexpr auto off = sec_conv(2);

			constexpr auto without_water = sec_conv(10 * 60);
			constexpr auto waiting_water = sec_conv(60 * 60);

			constexpr auto try_water = sec_conv(20 * 60);
		}
	}
	constexpr auto PRE_WASH = sec_conv(4);
	constexpr auto WASH = sec_conv(18);
	constexpr auto POST_WASH = sec_conv(3);

	constexpr auto PRE_FILLING = sec_conv(5);
	constexpr auto POST_FILLING = sec_conv(5);

	constexpr auto BEETWEN_FILLIG = sec_conv(60);
	constexpr auto MAX_PUMPING = sec_conv(3 * 60 * 60);
#endif // DEBUG
}


namespace pins {
	using namespace Mcucpp::IO;

	using LowSensor = Pd0;
	using HighSensor = Pd1;
	using ValveOpened = Pd2;
	using WaterLack = Pd3;

	using Pump = Pd4;
	using Valve = Pb5;
	using WashValve = Pb4;

	using Btn = Pe6;
}

//class Stats {
//	public:
//		template<typename Time = unsigned short int, typename Counter = unsigned short int>
//		struct Data {
//			private:
//				template<typename CordsT>
//				struct Point {
//					const CordsT x;
//					const CordsT y;
//
//			public:
//					friend bool operator<(const Point& l, const Point& r) {
//						return l.x < r.x && l.y < r.y;
//					}
//
//					friend bool operator>(const Point& l, const Point& r) {
//						return l.x > r.x && l.y > r.y;
//					}
//
//					friend bool operator==(const Point& l, const Point& r) {
//						return l.x == r.x && l.y == r.y;
//					}
//				};
//
//				template<typename CordsT>
//				struct Line {
//					const Point<CordsT> start;
//					const Point<CordsT> end;
//
//					template<typename S, typename E, typename V>
//					static bool val_in_range(S start, E end, V val) {
//						return val > min(start, end) && val < max(start, end);
//					}
//
//					template<typename T>
//					bool on_line(T val) const {
//						return on_line(Point<T>{val, 0});
//					}
//
//					template<typename T>
//					bool on_line(Point<T> val) const {
//						if (start.y == end.y) {
//							return val.y == end.y && val_in_range(start.x,  end.x, val.x);
//						}
//						else if (start.x == end.x) {
//							return val.x == end.x && val_in_range(start.y, end.y, val.y);
//						}
//
//						float k = (float)(end.y - start.y) / (end.x - start.x);
//						float c = (float)start.y - k * start.x;
//
//						return val.y == val.x * k + c;
//					}
//				
//					template<typename T>
//					bool check_collision(Line<T> val) const {
//						return val_in_range(start.x, end.x, val.start.x) || val_in_range(start.x, end.x, val.end.x);
//					}
//
//					template<typename T>
//					Line<T> get_collision(Line<T> val) const {
//						return { max(start.x, val.start.x),
//								 min(end.x, val.end.x) };
//					}
//
//					CordsT distance(void) const {
//						if (start.y == end.y) {
//							return abs(end.x - start.x);
//						}
//						else if (start.x == end.x) {
//							return abs(end.y - start.y);
//						}
//						return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
//					}
//				};
//
//				template<typename CordsT>
//				struct Triangle {
//					private:
//						typedef Point<CordsT> point;
//					public:
//						point a, b, h;
//
//						template<typename T>
//						Triangle(Line<T> a, T S): a(a.start), b(a.end), h((S * 2) / a.size()) {}
//
//						template<typename T>
//						Triangle(Data a, T S) : a(a.start()), b(a.end()), h((S * 2) / (a.Derr() <<  1)) {}
//				};
//
//				Triangle<Time> triangle() const {
//					return Triangle<Time>(*this, 100);
//				}
//
//			public:
//				Time time = 0;
//				Counter counts = 0;
//
//				Line<Time> operator()(void) const {
//					return {start(), end()};
//				}
//
//				Time Derr(void) const {
//					return counts >> 1; // counts * (1 / 2)
//				}
//
//				Time start(void) const {
//					return time - Derr();
//				}
//
//				Time end(void) const {
//					return time + Derr();
//				}
//
//				friend bool operator<(const Data& l, const Data& r) {
//					Line<Time> l_line = l(), r_line =  r();
//					if (l_line.check_collision(r_line)) {
//						Line<Time> coll_line = l_line.get_collision(r_line);
//						Triangle<Time> l_triagle = l.triangle(), r_triagle = r.triangle();
//
//					}
//					else {
//						return l.time < r.time;
//					}
//				}
//		};
//
//		bool freeze = true;
//
//	private:
//		typedef Data<unsigned short int, unsigned char> DataType;
//		//typedef Data<> StagingType;
//
//		MedianFilteredValue < DataType, 10, DataType{0, 0} > current;
//		DataType staging;
//
//	public:
//		DataType operator()(void) {
//			return current();
//		}
//
//		void new_iteration(void) {
//			if (staging.counts > 2) {
//				current(staging);
//			}
//			staging = { 0, 0 };
//		}
//
//		void interruptCallback(void) {
//			if (!freeze) {
//				overflow_limit_sum(staging.counts, 1);
//			}
//		}
//
//		void timerCallback(void) {
//			if (!freeze) {
//				overflow_limit_sum(staging.time, 1);
//			}
//		}
//};

//ISRv(INT2_vect);

namespace sensors {

	namespace classes {
		using namespace pins;

		using High = Sensor<HighSensor, unsigned char, delay::treshold::high::on, delay::treshold::high::off>;
		using Low = Sensor<LowSensor, unsigned char, delay::treshold::low::on, delay::treshold::low::off>;
		
		class Valve : public Sensor<ValveOpened, unsigned char, delay::treshold::valve::on, delay::treshold::valve::off> {
			public:
				class Stats {
					public:
						bool freeze = true;

					private:
						MedianFilteredValue <unsigned short int, 10> current;
						unsigned short int staging;

					public:
						unsigned short int operator()(void) {
							return current();
						}

						operator bool(void) {
							return operator()() > 0 && operator()() < staging;
						}

						void new_iteration(void) {
							current(staging);
							staging = 0;
						}

						void timerCallback(void) {
							if (!freeze) {
								overflow_limit_sum(staging, 1);
							}
						}
				} stats;

			protected:
				typedef Sensor<ValveOpened, unsigned char, delay::treshold::valve::on, delay::treshold::valve::off> base;

			public:
				Valve(void): base(false) {}

				void operator()(void) override {
					base::operator()();
					if (real_val()) {
						stats.timerCallback();
					}
				}
		};

		class WaterLack : public Sensor<pins::WaterLack, unsigned int, delay::treshold::lack::on, delay::treshold::lack::off> {
			protected:
				typedef Sensor<pins::WaterLack, unsigned int, delay::treshold::lack::on, delay::treshold::lack::off> base;
				//friend ISRf(INT3_vect);

				bool normal_mode;

				const unsigned int get_timer_delay(const bool state) const override {
					return !this->normal_mode & state ? delay::treshold::lack::waiting_water : base::get_timer_delay(state);
				}

				void handler_timer_done(void) override {
					if (this->stage_val != this->value()) {
						base::handler_timer_done();
						this->normal_mode = true;
						if (!this->value()) {
							this->timer(delay::treshold::lack::without_water);
						}

					}
					else if (!this->value()) {
						this->normal_mode = false;
					}
				}
				
			public:
				WaterLack(void) :
					base(true), normal_mode(true) {};

		};
	}

	classes::High High(false);
	classes::Low Low(false);
	classes::Valve Valve;
	classes::WaterLack WaterLack;
	
}

class Pump: public TimerBase<unsigned int> {
	protected:
		#ifdef WDT_vect
				friend ISRf(WDT_vect);
		#endif
		friend int main();

		MedianFilteredValue <unsigned short int, 10> stat;

		void handler_timer_done(void) override {
			switch (this->state) {
				case States::Working:
					this->state = States::OverPumping;
					break;

				case States::Cooldown:
					this->state = States::Ready;
					break;
			}
		}

		unsigned short int pump_time(void) {
			return stat() == 0 ? delay::MAX_PUMPING : (unsigned short int)(((double)stat() * 1.3) + 0.5);
		}

		unsigned short int work_time(void) {
			return pump_time() - this->time;
		}

	public:
		void reset_timer(void) {
			this->timer(pump_time());
		}

		void new_iteration(void) {
			stat(work_time());
		}
	
		enum States: const unsigned char {
			Ready,
			OverPumping,
			Cooldown,
			Working
		};
		unsigned char state;

		Pump(void): TimerBase<unsigned int>(), state(States::Ready) {
			pins::Pump::SetConfiguration(pins::Pump::Port::Out);
			pins::Pump::Clear();
		}

		void on(void) {
			if (!pins::Pump::IsSet()) {
				pins::Pump::Set();
				this->state = States::Working;
				this->reset_timer();
			}
		};

		void off(void) {
			if (pins::Pump::IsSet()) {
				pins::Pump::Clear();
				this->state = States::Cooldown;
				const unsigned short int work_time = this->work_time();
				this->timer((delay::treshold::lack::off <= work_time && work_time < delay::treshold::lack::off + sec_conv(8)) ? delay::treshold::lack::try_water : delay::BEETWEN_FILLIG);
			}
		}
} pump;

class Tank {
	public:
		enum TankStates :const unsigned char {
			TankEmpty = 0x03,
			TankNormal = 0x02,
			TankFull = 0,
		};
		//Sensor* const high, * const low, * const valve;
	
		static TankStates status(void) {
			static TankStates buffer;
			TankStates state = static_cast <TankStates>((unsigned char)(((sensors::High.value() ? 0x01 : 0x00) << 1) | (sensors::Low.value() ? 0x01 : 0x00)));
			if (state == TankFull) {
				buffer = state;
			}
			else if (!sensors::Valve.value()) {
				buffer = state;
			}
			return buffer;
		};
};

Timer<unsigned char> timer;

namespace wd {
	callback* const callbacks[]{ &sensors::High, &sensors::Low, &sensors::Valve, &sensors::WaterLack, &pump, &timer};
	constexpr unsigned char const count = (sizeof(callbacks) / sizeof(callbacks[0])) - 1;
	volatile unsigned char counter = 0;
}

template<typename Port, unsigned char mask = 0xff>
void hizPort(void) {
	Port::SetConfiguration(mask, Port::In);
	Port::SetPullUp(mask, Port::PullUp);
}

template<class pin>
inline bool pin_state(void) {
	return pin::Port::Read() & bit(pin::Number);
}

enum States : const unsigned char {
	ALERT,
	STANDBY,
	PRE_WASH,
	WASH,
	POST_WASH,
	PRE_FILL,
	FILL,
	POST_FILL,
};

States state = STANDBY;

enum delaysID : const unsigned char {
	PRE_WASH_DELAY,
	WASH_DELAY,
	POST_WASH_DELAY,
	PRE_FILL_DELAY,
	POST_FILL_DELAY,
};

namespace task {
	class Alert {
	public:
		Alert(void) {
			pump.off();
			pins::Valve::Clear();
			pins::WashValve::Clear();
		}

		operator bool(void) {
			return true;
		}
	};

	class Stanby {
	public:
		/*Stanby(void) {
			pump.off();
			pins::Valve::Clear();
			pins::WashValve::Clear();
		}*/

		~Stanby(void) {
			sensors::Valve.stats.freeze = true;
			state = PRE_WASH;
		}

		operator bool(void) {
			return !((Tank::status() == Tank::TankEmpty || sensors::Valve.stats) && sensors::WaterLack.value() && pump.state == pump.Ready);
		}
	};

	class PreWash {
	public:
		PreWash(void) {
			//pump.off();
			pins::WashValve::Set();
			pins::Valve::Set();
			//pins::WashValve::Clear();
		}

		~PreWash(void) {
			state = WASH;
		}

		operator bool(void) {
			return !timer(PRE_WASH_DELAY, delay::PRE_WASH);
		}
	};

	class Wash {
	public:
		Wash(void) {
			//pins::Valve::Set();
			//pins::WashValve::Set();
			pins::Pump::Set();
		}

		~Wash(void) {
			//pins::WashValve::Clear();
			pins::Pump::Clear();
			state = POST_WASH;
		}

		operator bool(void) {
			return !timer(WASH_DELAY, delay::WASH);
		}
	};

	class PostWash {
	public:
		~PostWash(void) {
			//pins::WashValve::Clear();
			pins::WashValve::Clear();
			state = PRE_FILL;
		}

		operator bool(void) {
			return !timer(POST_WASH_DELAY, delay::POST_WASH);
		}
	};

	class PreFill {
	public:
		/*PreFill(void) {
			pins::Valve::Set();
		}*/

		~PreFill(void) {
			state = FILL;
		}

		operator bool(void) {
			return !timer(PRE_FILL_DELAY, delay::PRE_FILLING);
		}
	};

	class Fill {
	public:
		Fill(void) {
			pump.on();
		}

		~Fill(void) {
			pump.off();
			if (state == POST_FILL) {
				pump.new_iteration();
			}
		}

		operator bool(void) {
			if (Tank::status() == Tank::TankFull) {
				sensors::Valve.stats.freeze = false;
				state = POST_FILL;
				return false;
			}
			else if (!sensors::WaterLack.value()) {
				state = POST_FILL;
				return false;
			}
			else {
				if (pump.state == pump.OverPumping) {
					state = ALERT;
					return false;
				}
				else if (sensors::Valve.value()) {
					pump.reset_timer();
				}
			}
			return true;
		}
	};

	class PostFill {
	public:
		/*PostFill(void) {
			pins::Valve::Set();
		}*/

		~PostFill(void) {
			pins::Valve::Clear();
			sensors::Valve.stats.new_iteration();
			state = STANDBY;
		}

		operator bool(void) {
			return !timer(POST_FILL_DELAY, delay::POST_FILLING);
		}
	};
}

template<typename T>
void process_task(T t) {
	for (; t;) {
		sleep_mode();
	}
}


void __attribute__((used, naked, section(".init1")))
pre_init(void) {
	MCUCR = 0x00;

	pins::Valve::SetConfiguration(pins::Valve::Port::Out);
	pins::Valve::Clear();

	pins::WashValve::SetConfiguration(pins::WashValve::Port::Out);
	pins::WashValve::Clear();

#ifdef DEBUG
	bitSet(DDRB, PINB0);
	bitSet(DDRD, PIND5);
#else
	hizPort<Mcucpp::IO::Portb, 0xCF>();
	hizPort<Mcucpp::IO::Portd, 0xE0>();
	hizPort<Mcucpp::IO::Portc>();
	hizPort<Mcucpp::IO::Porte>();
	hizPort<Mcucpp::IO::Portf>();
#endif // DEBUG

	bitClear(ADCSRA, ADEN);
	bitSet(ACSR, ACD);

	PRR0 = bit(PRTWI) | bit(PRTIM0) | bit(PRTIM1) | bit(PRSPI) | bit(PRADC);
	PRR1 = bit(PRUSB) | bit(PRTIM3) | bit(PRUSART1);
}

void __attribute__((used, naked, section(".init8")))
init(void) {
	wdt_reset(); //asm("wdr");
	WDTCSR |= bit(WDCE) | bit(WDE);
	WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1); //bit(WDP2);

	// Enable Interrupt any edge
	EICRA = bit(ISC30) | bit(ISC20) | bit(ISC10) | bit(ISC00);
	EIMSK = bit(INT0) | bit(INT1) | bit(INT2) | bit(INT3);


	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

int main(void) {
	for (;;) {
		#ifdef DEBUG
				bitWrite(PORTB, PINB0, Tank::status() == Tank::TankEmpty);
				bitWrite(PORTD, PIND5, sensors::High.value() && sensors::Low.value());
		#endif // DEBUG

		switch (state) {
			case ALERT:
				process_task(task::Alert());
				break;

			case STANDBY:
				process_task(task::Stanby());
				break;

			case PRE_WASH:
				process_task(task::PreWash());
				break;

			case WASH:
				process_task(task::Wash());
				break;

			case POST_WASH:
				process_task(task::PostWash());
				break;

			case PRE_FILL:
				process_task(task::PreFill());
				break;

			case FILL:
				process_task(task::Fill());
				break;

			case POST_FILL:
				process_task(task::PostFill());
				break;
		}
		//wdt_reset(); //asm("wdr");
		//sleep_mode();
	}
}

//void __attribute__((used, naked, section(".fini0")))
//terminate(void) {
//	cli();
//}

ISRf(INT0_vect) {
	sensors::Low.interruptCallback();
}

ISRf(INT1_vect) {
	sensors::High.interruptCallback();
}

ISRf(INT2_vect) {
	sensors::Valve.interruptCallback();
}

ISRf(INT3_vect) {
	sensors::WaterLack.interruptCallback();
}

/*
ISRf(INT6_vect) {
	state = (state == ALERT) ? PRE_FILLING : ALERT;
}
*/

ISRf(WDT_vect) {
	wd::callbacks[wd::counter]->operator()();
	if (wd::counter < wd::count) {
		wd::counter++;
	}
	else {
		wd::counter = 0;
	}
	bitSet(WDTCSR, WDIE);
}