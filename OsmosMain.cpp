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
#include <math.h>

FUSES = {
	.low = FUSE_CKSEL0, // 0xFE,
	.high = FUSE_BOOTRST & FUSE_SPIEN & FUSE_BOOTSZ1 & FUSE_BOOTSZ0, // 0xD8,
	.extended = FUSE_BODLEVEL2 // 0xFB
};

//#define DEBUG 1

#define watchdog 8
#define wd_calls 6
#define sec_conv(val) (unsigned int)floor( ((((double)(val) - ((double)watchdog / (wd_calls * 2))) * watchdog) / wd_calls) + 0.5 )

#ifdef DEBUG
	#define DEBUG_LEVEL 1

	#define HIGH_PRESURE_ON_TRESHOLD_PERIOD sec_conv(3)
	#define HIGH_PRESURE_OFF_TRESHOLD_PERIOD sec_conv(4)

	#define LOW_PRESURE_ON_TRESHOLD_PERIOD sec_conv(3)
	#define LOW_PRESURE_OFF_TRESHOLD_PERIOD sec_conv(4)

	#define VALVE_OPENED_ON_TRESHOLD_PERIOD sec_conv(4)
	#define VALVE_OPENED_OFF_TRESHOLD_PERIOD sec_conv(3)

	#define WATER_LACK_ON_TRESHOLD_PERIOD sec_conv(3)
	#define WATER_LACK_OFF_TRESHOLD_PERIOD sec_conv(3)

	#define WATER_LACK_TRESHOLD_WITHOUT_WATER sec_conv(15 * 60)
	#define WATER_LACK_WAITING_WATER sec_conv(60 * 60)

	#define WATER_LACK_TRY_WATER sec_conv(30 * 60)

	#define PRE_FILLING_DELAY sec_conv(15)
	#define POST_FILLING_DELAY sec_conv(2)

	#define BEETWEN_FILLIG_DELAY sec_conv(60)
	#define MAX_PUMPING_TIME sec_conv(4 * 60)


#else

	#define HIGH_PRESURE_ON_TRESHOLD_PERIOD sec_conv(4)
	#define HIGH_PRESURE_OFF_TRESHOLD_PERIOD sec_conv(2)

	#define LOW_PRESURE_ON_TRESHOLD_PERIOD sec_conv(4)
	#define LOW_PRESURE_OFF_TRESHOLD_PERIOD sec_conv(2)

	#define VALVE_OPENED_ON_TRESHOLD_PERIOD sec_conv(1)
	#define VALVE_OPENED_OFF_TRESHOLD_PERIOD sec_conv(2)

	#define WATER_LACK_ON_TRESHOLD_PERIOD sec_conv(2)
	#define WATER_LACK_OFF_TRESHOLD_PERIOD sec_conv(2)

	#define WATER_LACK_TRESHOLD_WITHOUT_WATER sec_conv(10 * 60)
	#define WATER_LACK_WAITING_WATER sec_conv(60 * 60)

	#define WATER_LACK_TRY_WATER sec_conv(30 * 60)

	#define PRE_FILLING_DELAY sec_conv(60)
	#define POST_FILLING_DELAY sec_conv(10)

	#define BEETWEN_FILLIG_DELAY sec_conv(60)
	#define MAX_PUMPING_TIME sec_conv(3 * 60 *60)


#endif // DEBUG


namespace pins {
	using namespace Mcucpp::IO;

	using LowSensor = Pd0;
	using HighSensor = Pd1;
	using ValveOpened = Pd2;
	using WaterLack = Pd3;

	using Pump = Pd4;
	using Valve = Pb5;

	using Btn = Pe6;
}

namespace sensors {

	namespace classes {
		using namespace pins;

		using High = Sensor<HighSensor, unsigned char, HIGH_PRESURE_ON_TRESHOLD_PERIOD, HIGH_PRESURE_OFF_TRESHOLD_PERIOD>;
		using Low = Sensor<LowSensor, unsigned char, LOW_PRESURE_ON_TRESHOLD_PERIOD, LOW_PRESURE_OFF_TRESHOLD_PERIOD>;
		using Valve = Sensor<ValveOpened, unsigned char, VALVE_OPENED_ON_TRESHOLD_PERIOD, VALVE_OPENED_OFF_TRESHOLD_PERIOD>;

		class WaterLack : public Sensor<pins::WaterLack, unsigned int, WATER_LACK_ON_TRESHOLD_PERIOD, WATER_LACK_OFF_TRESHOLD_PERIOD> {
			protected:
				friend ISRf(INT3_vect);

				bool normal_mode;

				const unsigned int get_timer_delay(const bool state) const override {
					return !this->normal_mode & state ? WATER_LACK_WAITING_WATER : Sensor<pins::WaterLack, unsigned int, WATER_LACK_ON_TRESHOLD_PERIOD, WATER_LACK_OFF_TRESHOLD_PERIOD>::get_timer_delay(state);
				}

				void handler_timer_done(void) override {
					if (this->stage_val != this->value()) {
						Sensor<pins::WaterLack, unsigned int, WATER_LACK_ON_TRESHOLD_PERIOD, WATER_LACK_OFF_TRESHOLD_PERIOD>::handler_timer_done();
						this->normal_mode = true;
						if (!this->value()) {
							this->timer(WATER_LACK_TRESHOLD_WITHOUT_WATER);
						}

					}
					else if (!this->value()) {
						this->normal_mode = false;
					}
				}
				
			public:
				WaterLack(void) :
					Sensor<pins::WaterLack, unsigned int, WATER_LACK_ON_TRESHOLD_PERIOD, WATER_LACK_OFF_TRESHOLD_PERIOD>(true), normal_mode(true) {};

		};
	}

	classes::High High(false);
	classes::Low Low(false);
	classes::Valve Valve(false);
	classes::WaterLack WaterLack;
	
}

class Pump: public TimerBase<unsigned int> {
	protected:
		#ifdef WDT_vect
				friend ISRf(WDT_vect);
		#endif
				friend int main();

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

		void setTimerMaxPumpTime(void) {
			this->timer(MAX_PUMPING_TIME);
		}

	public:
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
				this->setTimerMaxPumpTime();
			}
		};

		void off(void) {
			if (pins::Pump::IsSet()) {
				pins::Pump::Clear();
				this->state = States::Cooldown;
				const unsigned int work_time = MAX_PUMPING_TIME - this->time;
				this->timer((WATER_LACK_OFF_TRESHOLD_PERIOD <= work_time && work_time < WATER_LACK_OFF_TRESHOLD_PERIOD + sec_conv(8)) ? WATER_LACK_TRY_WATER : BEETWEN_FILLIG_DELAY);
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

Timer<unsigned char> delay;

namespace wd {
	callback* const callbacks[]{ &sensors::High, &sensors::Low, &sensors::Valve, &sensors::WaterLack, &pump, &delay };
	constexpr unsigned char const count = (sizeof(callbacks) / sizeof(callbacks[0])) - 1;
	volatile unsigned char counter = 0;
}

template<typename Port, unsigned char mask = 0xff>
void hizPort(void) {
	Port::SetConfiguration(mask, Port::In);
	Port::SetPullUp(mask, Port::PullUp);
}

int main(void) {
	MCUCR = 0x00;

	pins::Valve::SetConfiguration(pins::Valve::Port::Out);
	pins::Valve::Clear();

	#ifdef DEBUG
		bitSet(DDRB, PINB0);
		bitSet(DDRD, PIND5);
	#else
		hizPort<Mcucpp::IO::Portb, 0xDF>();
		hizPort<Mcucpp::IO::Portd, 0xE0>();
		hizPort<Mcucpp::IO::Portc>();
		hizPort<Mcucpp::IO::Porte>();
		hizPort<Mcucpp::IO::Portf>();
	#endif // DEBUG

	wdt_reset(); //asm("wdr");
	WDTCSR |= bit(WDCE) | bit(WDE);
	WDTCSR = bit(WDE) | bit(WDIE) | bit(WDP0) | bit(WDP1); //bit(WDP2);


	bitClear(ADCSRA, ADEN);
	bitSet(ACSR, ACD);


	PRR0 = bit(PRTWI) | bit(PRTIM0) | bit(PRTIM1) | bit(PRSPI) | bit(PRADC);
	PRR1 = bit(PRUSB) | bit(PRTIM3) | bit(PRUSART1);
	// Enable Interrupt any edge
	EICRA = bit(ISC30) | bit(ISC20) | bit(ISC10) | bit(ISC00);
	EIMSK = bit(INT0) | bit(INT1) | bit(INT2) | bit(INT3);

	
	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	enum States : const unsigned char {
		ALERT,
		STANDBY,
		PRE_FILLING,
		FILLING,
		POST_FILLING,
	} state = STANDBY;

	enum delaysID : const unsigned char {
		PRE_FILL_DELAY = 1,
		POST_FILL_DELAY = 2,
	};

	for (;;) {
		#ifdef DEBUG
				bitWrite(PORTB, PINB0, Tank::status() == Tank::TankEmpty);
				bitWrite(PORTD, PIND5, sensors::High.value() && sensors::Low.value());
		#endif // DEBUG

		switch (state) {
			case ALERT:
				pump.off();
				pins::Valve::Clear();
				break;

			case STANDBY:
				if (Tank::status() == Tank::TankEmpty && sensors::WaterLack.value() && pump.state == pump.Ready) {
					state = PRE_FILLING;
				}
				break;

			case PRE_FILLING:
				pins::Valve::Set();
				if (delay(PRE_FILL_DELAY, PRE_FILLING_DELAY)) {
					state = FILLING;
				}
				break;

			case FILLING:
				if (!sensors::WaterLack.value() || Tank::status() == Tank::TankFull) {
					//pins::Pump::Clear();
					pump.off();
					state = POST_FILLING;
				}
				else {
					//pins::Pump::Set();
					pump.on();
					if (pump.state == pump.OverPumping) {
						state = ALERT;
					}
					else if (sensors::Valve.value()) {
						pump.setTimerMaxPumpTime();
					}
				}
				break;

			case POST_FILLING:
				if (delay(POST_FILL_DELAY, POST_FILLING_DELAY)) {
					pins::Valve::Clear();
					state = STANDBY;
				}
				break;
		}
		//wdt_reset(); //asm("wdr");
		sleep_mode();
	}
}

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