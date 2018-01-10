/** SynGenBalancedResLoad Example
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

#include "Simulation.h"

using namespace DPsim;

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
	//Real Rkq2 = 0.0237;
	//Real Llkq2 = 0.125;
	Real Llkq2 = 0;
	Real Rkq2 = 0;

	// Declare circuit components
	Components::Base::Ptr gen = std::make_shared<Components::EMT::SynchronGenerator>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	Components::Base::Ptr r1 = std::make_shared<Components::EMT::Resistor>("r1", 0, 1, loadRes);
	Components::Base::Ptr r2 = std::make_shared<Components::EMT::Resistor>("r2", 0, 2, loadRes);
	Components::Base::Ptr r3 = std::make_shared<Components::EMT::Resistor>("r3", 0, 3, loadRes);

	Components::Base::List circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Set up simulation
	Real om = 2.0*M_PI*60.0;
	Real tf = 0.1;
	Real dt = 0.000001;
	Real t = 0;
	Int downSampling = 25;
	Simulation newSim("EMT_SynGen_BalanceResLoad", circElements, om, dt, tf, Logger::Level::INFO, SimulationType::EMT, downSampling);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	auto genPtr = std::dynamic_pointer_cast<Components::EMT::SynchronGenerator>(gen);
	genPtr->initialize(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	// Calculate initial values for circuit at generator connection point
	//Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	//Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	//Real initPowerFactor = acos(initActivePower / initApparentPower);
	//Real initVolt1 = initTerminalVolt * cos(initVoltAngle);
	//Real initVolt2 = initTerminalVolt * cos(initVoltAngle - 2 * M_PI / 3);
	//Real initVolt3 = initTerminalVolt * cos(initVoltAngle + 2 * M_PI / 3);
	//Real initCurrent1 = initTerminalCurr * cos(initVoltAngle + initPowerFactor);
	//Real initCurrent2 = initTerminalCurr * cos(initVoltAngle + initPowerFactor - 2 * M_PI / 3);
	//Real initCurrent3 = initTerminalCurr * cos(initVoltAngle + initPowerFactor + 2 * M_PI / 3);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;

	return 0;
}
