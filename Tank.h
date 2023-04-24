#pragma once

#ifndef TANK_H_
#define TANK_H_

class Tank {
	public:
		enum class State :const uint8_t {
			Empty =			0b11u,
			Normal =		0b10u,
			StrangeFull =	0b01u,
			Full =			0b0u,
		};

		static State state(void) {
			static State buffer;
			const State state = static_cast <State>((uint8_t)(((sensor::High.value() ? 0x01u : 0x00u) << 1u) | (sensor::Low.value() ? 0x01u : 0x00u)));

			memory();

			/*switch (state) {
				case State::StrangeFull:
					buffer = State::Full;
					break;
				case State::Empty:
					if (!sensor::Valve) {
						break;
					}
				case State::Full:
					buffer = state;
					break;
				default:
					if (!sensor::Valve.value()) {
						buffer = state;
					}
					break;
			}*/

			/*if (state == State::Full) {
				buffer = state;
			}
			else if (state == State::StrangeFull) {
				buffer = State::Full;
			}
			else if (!sensor::Valve.value()) {
				buffer = state;
			}
			else if (state == State::Empty && sensor::Valve) {
				buffer = state;
			}*/

			if (state == State::StrangeFull) {
				buffer = State::Full;
			}
			else if (state == State::Full || !sensor::Valve.value() || (state == State::Empty && sensor::Valve)) {
				buffer = state;
			}
			return buffer;
		};
};

#endif