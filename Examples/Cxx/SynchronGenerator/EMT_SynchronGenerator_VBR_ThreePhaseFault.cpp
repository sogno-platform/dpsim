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

#include "DPsim.h"

using namespace DPsim;
using namespace CPS::Components::EMT;

int main(int argc, char* argv[])
{
	// Define machine parameters in per unit
	Real nomPower = 555e6;
	Real nomPhPhVoltRMS = 24e3;
	Real nomFreq = 60;
	Real nomFieldCurr = 1300;
	Int poleNum = 2;
	Real J = 2.8898e+04;
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

	// Declare circuit components
	ComponentBase::Ptr gen = SynchronGeneratorVBR::make("gen", 0, 1, 2,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Logger::Level::INFO);

	Real loadRes = 1.92;
	ComponentBase::Ptr r1 = Resistor::make("r1", 0, DEPRECATEDGND, loadRes);
	ComponentBase::Ptr r2 = Resistor::make("r2", 1, DEPRECATEDGND, loadRes);
	ComponentBase::Ptr r3 = Resistor::make("r3", 2, DEPRECATEDGND, loadRes);

	SystemTopology system(60);
	system.mComponents = { gen, r1, r2, r3 };

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	ComponentBase::Ptr rBreaker1 = Resistor::make("rbreak1", 0, DEPRECATEDGND, breakerRes);
	ComponentBase::Ptr rBreaker2 = Resistor::make("rbreak2", 1, DEPRECATEDGND, breakerRes);
	ComponentBase::Ptr rBreaker3 = Resistor::make("rbreak3", 2, DEPRECATEDGND, breakerRes);

	SystemTopology systemBreakerOn(60);
	systemBreakerOn.mComponents = {gen, rBreaker1, rBreaker2, rBreaker3, r1, r2, r3 };

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.00001; t = 0;
	Int downSampling = 50;
	Simulation sim("EMT_SynchronGenerator_VBR", system, dt, tf, Solver::Domain::EMT);
	sim.setLogDownsamplingRate(downSampling);
	sim.addSystemTopology(systemBreakerOn);

	// Initialize generator
	Real initActivePower = 300e6;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	auto genPtr = std::dynamic_pointer_cast<Components::EMT::SynchronGeneratorVBR>(gen);
	genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	sim.setSwitchTime(0.1, 1);
	sim.setSwitchTime(0.2, 0);

	sim.run();

	return 0;
}
