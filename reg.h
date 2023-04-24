#pragma once

#ifndef REGS_H_
#define REGS_H_
#include "IOReg.h"

namespace reg {
	using eimsk = IOReg<0x1D + __SFR_OFFSET>;
	using eifr = IOReg<0x1C + __SFR_OFFSET>;
	using wdtcsr = IOReg<0x60>;
	using pcmsk0 = IOReg<0x6B>;
	using pcifr = IOReg<0x1B + __SFR_OFFSET>;
}

/*
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
*/

#endif