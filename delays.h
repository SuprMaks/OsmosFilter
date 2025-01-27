#pragma once

#ifndef DELAYS_CONF_H_
#define DELAYS_CONF_H_

constexpr uint8_t watchdog = 8u;
constexpr uint8_t wd_calls = 7u;

constexpr auto sensor_tick_rate = 1.0 / ((double)wd_calls / watchdog);

template<typename outT = uint16_t, typename T>
constexpr outT sec_conv(T val) {
	return (outT)simple_round(((double)val * sensor_tick_rate) + 1u);
};

namespace delay {
#ifdef DEBUG
#define DEBUG_LEVEL 1
#else
	namespace threshold {
		namespace high {
			constexpr auto on = sec_conv(4u);
			constexpr auto off = sec_conv(2u);
		}

		namespace low {
			constexpr auto on = sec_conv(4u);
			constexpr auto off = sec_conv(2u);
		}

		namespace valve {
			constexpr auto on = 0u;
			constexpr auto off = sec_conv(2u);
		}

		namespace lack {
			constexpr auto on = sec_conv(2u);
			constexpr auto off = sec_conv(8u);

			constexpr auto without_water = sec_conv(20u * 60u);
			constexpr auto waiting_water = sec_conv(60u * 60u);

			constexpr auto try_water = sec_conv(30u);
			constexpr auto try_water_max = try_water << 6u;
		}
	}
	constexpr auto WASH_BASE = 20u;

	constexpr auto PRE_WASH = sec_conv(4u);
	constexpr auto FAST_WASH = sec_conv(WASH_BASE);
	constexpr auto WASH = sec_conv(WASH_BASE * 2u);
	constexpr auto AUTO_WASH_LIMIT = sec_conv(WASH_BASE * 2u * 5u);
	constexpr auto POST_WASH = sec_conv(4u);

	constexpr auto PRE_FILLING = sec_conv(4u);
	constexpr auto FINISH = sec_conv(4u);

	constexpr auto MAX_PUMPING = sec_conv(3u * 60u * 60u);

	constexpr auto POST_FILL_WASH = sec_conv(WASH_BASE / 2.5);

	constexpr auto FAST_WASH_PERIOD = sec_conv(60u * 60u);
	constexpr auto AUTO_WASH_PERIOD = sec_conv(6u * 60u * 60u);

	constexpr auto BTN_TIMER = sec_conv(1u);
	constexpr auto BTN_LONG_TIMER = sec_conv(3u);
	constexpr auto BTN_LONG_LONG_TIMER = sec_conv(6u);
#endif // DEBUG
}

#endif