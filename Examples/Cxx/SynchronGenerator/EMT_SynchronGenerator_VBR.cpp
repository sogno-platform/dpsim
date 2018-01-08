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
	Real Rkq2 = 0;
	Real Llkq2 = 0;

	// Declare circuit components
	Components::Base::Ptr gen = std::make_shared<Components::EMT::SynchronGeneratorVBR>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, true);

	Real loadRes = 1037.8378;
	Components::Base::Ptr r1 = std::make_shared<Components::EMT::Resistor>("r1", 1, 0, loadRes);
	Components::Base::Ptr r2 = std::make_shared<Components::EMT::Resistor>("r2", 2, 0, loadRes);
	Components::Base::Ptr r3 = std::make_shared<Components::EMT::Resistor>("r3", 3, 0, loadRes);

	Components::Base::List circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	Components::Base::Ptr rBreaker1 = std::make_shared<Components::EMT::Resistor>("rbreak1", 1, 0, breakerRes);
	Components::Base::Ptr rBreaker2 = std::make_shared<Components::EMT::Resistor>("rbreak2", 2, 0, breakerRes);
	Components::Base::Ptr rBreaker3 = std::make_shared<Components::EMT::Resistor>("rbreak3", 3, 0, breakerRes);
	Components::Base::List circElementsBreakerOn;
	circElementsBreakerOn.push_back(rBreaker1);
	circElementsBreakerOn.push_back(rBreaker2);
	circElementsBreakerOn.push_back(rBreaker3);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.000001; t = 0;
	Int downSampling = 50;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT, downSampling);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);
	newSim.addSystemTopology(circElementsBreakerOn);
	newSim.switchSystemMatrix(0);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	auto genPtr = std::dynamic_pointer_cast<Components::EMT::SynchronGeneratorVBR>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;
	newSim.setSwitchTime(0.1, 1);
	newSim.setSwitchTime(0.2, 0);

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorVBR(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;

	return 0;
}
