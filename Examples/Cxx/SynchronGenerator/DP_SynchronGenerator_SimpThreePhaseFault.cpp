/** Synchron Generator Tests
 *
 * @file
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
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real nomFieldCurr = 1300;
	Int poleNum = 2;
	Real H = 3.7;

#if 0
	Real Ka = 20;
	Real Ta = 0.2;
	Real Ke = 1;
	Real Te = 0.314;
	Real Kf = 0.063;
	Real Tf = 0.35;
	Real Tr = 0.02;
#endif

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

	Real Ld_s = 0.23;
	Real Lq_s = 0.25;

	// Set up simulation
	Real om = 2.0*M_PI*60.0;
	Real tf = 0.3;
	Real dt = 0.00005;
	Int downSampling = 50;

	Real Ra = (Ld_s + Lq_s) / dt;
	// Declare circuit components
	String mGeneratorName = "DP_DqSimplified_" + std::to_string(dt);
	Component::Ptr gen = SynchronGeneratorDQSmpl::make(mGeneratorName, 0, 1, 2,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Ra);
	Real loadRes = 24e3*24e3 / 300e6;
	Component::Ptr r1 = Resistor::make("r1", 0, DEPRECATEDGND, loadRes);
	Component::Ptr r2 = Resistor::make("r2", 1, DEPRECATEDGND, loadRes);
	Component::Ptr r3 = Resistor::make("r3", 2, DEPRECATEDGND, loadRes);

	SystemTopology system(60);
	system.mComponents = { gen, r1, r2, r3 };

	// Declare circuit components for resistance change
	//Real breakerRes = 0.001;
	Real breakerRes = 19.2 + 0.001;
	Component::Ptr rBreaker1 = Resistor::make("rbreak1", 0, DEPRECATEDGND, breakerRes);
	Component::Ptr rBreaker2 = Resistor::make("rbreak2", 1, DEPRECATEDGND, breakerRes);
	Component::Ptr rBreaker3 = Resistor::make("rbreak3", 2, DEPRECATEDGND, breakerRes);

	SystemTopology systemBreakerOn(60);
	systemBreakerOn.mComponents = { gen, rBreaker1, rBreaker2, rBreaker3, r1, r2, r3 };

	Simulation sim("DP_SynchronGeneratorDQ_SimpThreePhaseFault", system, dt, tf);
	sim.setLogDownsamplingRate(downSampling);
	sim.addSystemTopology(systemBreakerOn);

	// Initialize generator
	Real initActivePower = 300e6;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 300e6;
	auto genPtr = std::dynamic_pointer_cast<SynchronGeneratorDQSmpl>(gen);
	genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);
	//genPtr->addExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);

	// Calculate initial values for circuit at generator connection point
#if 0
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);
#endif

	sim.setSwitchTime(0.1, 1);
	sim.setSwitchTime(0.2, 0);

	sim.run();

	return 0;
}
