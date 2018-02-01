/** SynGenVBRDP Example
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

#include "SynGenSimulation.h"
#include "Components.h"

using namespace DPsim;
using namespace DPsim::Components::DP;

int main(int argc, char* argv[])
{
	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv");

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
	Component::Ptr gen = SynchronGeneratorVBRNew::make("gen", 0, 1, 2,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, Logger::Level::INFO);
	Real loadRes = 1037.8378;
	Component::Ptr r1 = Resistor::make("r1", 0, GND, loadRes);
	Component::Ptr r2 = Resistor::make("r2", 1, GND, loadRes);
	Component::Ptr r3 = Resistor::make("r3", 2, GND, loadRes);

	Component::List comps = { gen, r1, r2, r3 };

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	Component::Ptr rBreaker1 = Resistor::make("rbreak1", 0, GND, breakerRes);
	Component::Ptr rBreaker2 = Resistor::make("rbreak2", 1, GND, breakerRes);
	Component::Ptr rBreaker3 = Resistor::make("rbreak3", 2, GND, breakerRes);

	Component::List compsBreakerOn = { gen, rBreaker1, rBreaker2, rBreaker3, r1, r2, r3 };

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.00001; t = 0;
	Int downSampling = 1;
	SynGenSimulation sim("DP_SynchronGenerator_VBR", comps, om, dt, tf, Logger::Level::INFO, SimulationType::DP, downSampling);
	sim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);
	sim.addSystemTopology(compsBreakerOn);
	sim.switchSystemMatrix(0);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	auto genPtr = std::dynamic_pointer_cast<Components::DP::SynchronGeneratorVBRNew>(gen);
	genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << sim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << sim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << sim.getRightSideVector() << std::endl;

	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;
	sim.setSwitchTime(0.1, 1);
	sim.setSwitchTime(0.2, 0);

	sim.run();

	return 0;
}
