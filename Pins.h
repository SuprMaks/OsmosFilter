#pragma once
#include "iopins.h"
#include "pinlist.h"

namespace pin {
	using namespace Mcucpp::IO;

#pragma region MC14490 pre filtered inputs
	using HighSensor = Pd0Inv;  // Dout //6
	using WaterLack = Pd1Inv; // Fout //7
	using ValveOpened = Pd2Inv; // Aout //3
	using LowSensor = Pd3Inv; //  Bout //4
	
	using Btn = Pb0Inv; // Eout //5
#pragma endregion

	using Pump = Pb6; //1
	using Valve = Pd4; //8
	using WashValve = Pb4; //2

	using Buzzer = Pb5;

	using FlowReg = Pd7; //9

	//using uart_tx = Pb3;
}