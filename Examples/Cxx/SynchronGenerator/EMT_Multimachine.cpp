/** SynGenVBR Example
*
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
*
* DPsim
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS::EMT::Ph3;

int main(int argc, char* argv[]) {
	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real nomFieldCurr = 1300;
	Int poleNum = 2;
	//Real J = 2.8898e+04;
	Real H = 3.7;

	Real Rs = 0.003;
	Real Ll = 0.15;
	Real Lmd = 1.6599;
	Real Lmd0 = 1.6599;
	Real Lmq = 1.61;
	Real Lmq0 = 1.61;
	Real Rfd = 0.0006;
	Real Llfd = 0.1648;
	Real Rkd = 0.0284;
	Real Llkd = 0.1713;
	Real Rkq1 = 0.0062;
	Real Llkq1 = 0.7252;
	Real Rkq2 = 0.0237;
	Real Llkq2 = 0.125;
	//Real Rkq2 = 0;
	//Real Llkq2 = 0;

	//Exciter
	Real Ka = 46;
	Real Ta = 0.06;
	Real Ke = -0.043478260869565223;
	Real Te = 0.46;
	Real Kf = 0.1;
	Real Tf = 1;
	Real Tr = 0.02;

	// Turbine
	Real Ta_t = 0.3;
	Real Fa = 0.3;
	Real Tb = 7;
	Real Fb = 0.3;
	Real Tc = 0.2;
	Real Fc = 0.4;
	Real Tsr = 0.1;
	Real Tsm = 0.3;
	Real Kg = 20;;

	// Set up simulation
	Real om = 2.0*M_PI*60.0;
	Real dt = 0.0005;
	Real tf = 0.3;

	// Declare circuit components
	String mGeneratorName = "EMT_VBR_1" + std::to_string(dt);
	Component::Ptr gen = SynchronGeneratorVBR::make(mGeneratorName, 0, 1, 2,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Logger::Level::INFO);

	// Declare circuit components
	String mGeneratorName2 = "EMT_VBR_2" + std::to_string(dt);
	Component::Ptr gen2 = SynchronGeneratorVBR::make(mGeneratorName2, 12, 13, 14,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Logger::Level::INFO);

	Real loadRes = 0.96;
	Real lineRes = 0.032;
	Real lindeInd = 0.35 / (2 * PI * 60);
	Real Res = 1e10;

	//Line Resistance
	Component::Ptr LineR1 = Resistor::make("LineR1", 0, 3, lineRes);
	Component::Ptr LineR2 = Resistor::make("LineR2", 1, 4, lineRes);
	Component::Ptr LineR3 = Resistor::make("LineR3", 2, 5, lineRes);

	//Line Resistance
	Component::Ptr Res1 = Resistor::make("Res1", 0, 6, Res);
	Component::Ptr Res2 = Resistor::make("Res2", 1, 7, Res);
	Component::Ptr Res3 = Resistor::make("Res3", 2, 8, Res);

	//Line Inductance
	Component::Ptr LineL1 = Inductor::make("LineL1", 3, 6, lindeInd);
	Component::Ptr LineL2 = Inductor::make("LineL2", 4, 7, lindeInd);
	Component::Ptr LineL3 = Inductor::make("LineL3", 5, 8, lindeInd);

	//Load
	Component::Ptr r1 = Resistor::make("r1", 6, -1, loadRes);
	Component::Ptr r2 = Resistor::make("r2", 7, -1, loadRes);
	Component::Ptr r3 = Resistor::make("r3", 8, -1, loadRes);

	//Line Inductance2
	Component::Ptr LineL12 = Inductor::make("LineL12", 6, 9, lindeInd);
	Component::Ptr LineL22 = Inductor::make("LineL22", 7, 10, lindeInd);
	Component::Ptr LineL32 = Inductor::make("LineL32", 8, 11, lindeInd);

	//Line Resistance2
	Component::Ptr LineR12 = Resistor::make("LineR12", 9, 12, lineRes);
	Component::Ptr LineR22 = Resistor::make("LineR22", 10, 13, lineRes);
	Component::Ptr LineR32 = Resistor::make("LineR32", 11, 14, lineRes);

	//Line Resistance
	Component::Ptr Res12 = Resistor::make("Res12", 6, 12, Res);
	Component::Ptr Res22 = Resistor::make("Res22", 7, 13, Res);
	Component::Ptr Res32 = Resistor::make("Res32", 8, 14, Res);

	SystemTopology system(60);
	system.mComponents = { gen, gen2, LineR1, LineR2, LineR3, LineL1, LineL2, LineL3, r1, r2, r3,
		Res1, Res2, Res3, LineR12, LineR22, LineR32, LineL12, LineL22, LineL32, Res12, Res22, Res32 };

	// Declare circuit components for resistance change
	Real breakerRes = 9.6 + 0.0001;
	Component::Ptr rBreaker1 = Resistor::make("rbreak1", 6, -1, breakerRes);
	Component::Ptr rBreaker2 = Resistor::make("rbreak2", 7, -1, breakerRes);
	Component::Ptr rBreaker3 = Resistor::make("rbreak3", 8, -1, breakerRes);

	SystemTopology compsBreakerOn(60);
	compsBreakerOn.mComponents = { gen, gen2, LineR1, LineR2, LineR3, LineL1, LineL2, LineL3,
		r1,	r2, r3, Res1, Res2, Res3, LineR12, LineR22, LineR32, LineL12, LineL22, LineL32,
		Res12, Res22, Res32, rBreaker1, rBreaker2, rBreaker3, };

	String mSimulationName = "EMT_SynchronGenerator_VBR_Multimachine" + std::to_string(dt);
	Simulation sim(mSimulationName, system, dt, tf, Domain::EMT, Solver::Type::MNA, Logger::Level::INFO);
	sim.addSystemTopology(compsBreakerOn);

	// Initialize generator
	Real initActivePower = 285.89e6;
	Real initReactivePower = 51.261e6;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 285.89e6;
	auto genPtr = std::dynamic_pointer_cast<EMT::Ph3::SynchronGeneratorVBR>(gen);
	genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);
	genPtr->addExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);
	genPtr->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initActivePower / nomPower, mechPower / nomPower);

	auto genPtr2 = std::dynamic_pointer_cast<EMT::Ph3::SynchronGeneratorVBR>(gen2);
	genPtr2->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);
	genPtr2->addExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);
	genPtr2->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initActivePower / nomPower, mechPower / nomPower);

	sim.setSwitchTime(0.1, 1);
	sim.setSwitchTime(0.2, 0);

	sim.run();

	return 0;
}
