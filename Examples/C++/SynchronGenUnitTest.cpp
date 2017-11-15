/** Synchron Generator Unit Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#include "SynchronGenUnitTest.h"

#include "Simulation.h"
#include "Utilities.h"

using namespace DPsim;

#if 0
// Machine parameters - stator referred
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real nomFreq = 60;
Real nomFieldCurr = 1300;
Int P = 2;
Real J = 28897.6459179918;
Real Rs = 0.0031;
Real Ll = 0.0004129;
Real Lmd = 0.0046;
Real Lmq = 0.0044;
Real Rf = 0.0006226;
Real Llfd = 0.0004538;
Real Rkd = 0.0295;
Real Llkd = 0.0005;
Real Rkq1 = 0.0064;
Real Llkq1 = 0.002;
Real Rkq2 = 0.0246;
Real Llkq2 = 0.0003;

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
#endif

void DPsim::SynGenUnitTestBalancedResLoad() {

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
	ElementPtr gen = make_shared<SynchronGeneratorEMT>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, true);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorEMT>("r1", 0, 1, loadRes);
	ElementPtr r2 = make_shared<ResistorEMT>("r2", 0, 2, loadRes);
	ElementPtr r3 = make_shared<ResistorEMT>("r3", 0, 3, loadRes);

	ElementList circElements;
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
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	shared_ptr<SynchronGeneratorEMT> genPtr = std::dynamic_pointer_cast<SynchronGeneratorEMT>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

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
}

void DPsim::SynGenUnitTestPhaseToPhaseFault() {
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

	// Declare circuit components
	ElementPtr gen = make_shared<SynchronGeneratorEMT>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorEMT>("r1", 0, 1, loadRes);
	ElementPtr r2 = make_shared<ResistorEMT>("r2", 0, 2, loadRes);
	ElementPtr r3 = make_shared<ResistorEMT>("r3", 0, 3, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.01;
	ElementPtr rBreaker = make_shared<ResistorEMT>("rbreak", 1, 2, breakerRes);

	ElementList circElementsBreakerOn;
	circElementsBreakerOn.push_back(rBreaker);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.2; dt = 0.00005; t = 0;
	Int downSampling = 50;
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
	newSim.addSystemTopology(circElementsBreakerOn);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	shared_ptr<SynchronGeneratorEMT> genPtr = std::dynamic_pointer_cast<SynchronGeneratorEMT>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	// Calculate initial values for circuit at generator connection point
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}
}

void DPsim::SynGenUnitTestThreePhaseFault() {
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
	ElementPtr gen = make_shared<SynchronGeneratorEMT>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorEMT>("r1", 1, 0, loadRes);
	ElementPtr r2 = make_shared<ResistorEMT>("r2", 2, 0, loadRes);
	ElementPtr r3 = make_shared<ResistorEMT>("r3", 3, 0, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	ElementPtr rBreaker1 = make_shared<ResistorEMT>("rbreak1", 1, 0, breakerRes);
	ElementPtr rBreaker2 = make_shared<ResistorEMT>("rbreak2", 2, 0, breakerRes);
	ElementPtr rBreaker3 = make_shared<ResistorEMT>("rbreak3", 3, 0, breakerRes);
	ElementList circElementsBreakerOn;
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
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
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
	shared_ptr<SynchronGeneratorEMT> genPtr = std::dynamic_pointer_cast<SynchronGeneratorEMT>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	// Calculate initial values for circuit at generator connection point
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);

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
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}
}

void DPsim::SynGenDPUnitTestBalancedResLoad() {
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

	// Declare circuit components
	ElementPtr gen = make_shared<SynchronGeneratorDP>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorDP>("r1", 0, 1, loadRes);
	ElementPtr r2 = make_shared<ResistorDP>("r2", 0, 2, loadRes);
	ElementPtr r3 = make_shared<ResistorDP>("r3", 0, 3, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.1; dt = 0.000001; t = 0;
	Int downSampling = 50;
	Simulation newSim(circElements, om, dt, tf, log, downSampling);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	Real initActivePower = 555e3;
	Real initReactivePower = 0;
	Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	Real initVoltAngle = -DPS_PI / 2;
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	shared_ptr<SynchronGeneratorEMT> genPtr = std::dynamic_pointer_cast<SynchronGeneratorEMT>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	// Calculate initial values for circuit at generator connection point
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::SynGenDPUnitTestThreePhaseFault() {
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
	ElementPtr gen = make_shared<SynchronGeneratorDP>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorDP>("r1", 1, 0, loadRes);
	ElementPtr r2 = make_shared<ResistorDP>("r2", 2, 0, loadRes);
	ElementPtr r3 = make_shared<ResistorDP>("r3", 3, 0, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	ElementPtr rBreaker1 = make_shared<ResistorDP>("rbreak1", 1, 0, breakerRes);
	ElementPtr rBreaker2 = make_shared<ResistorDP>("rbreak2", 2, 0, breakerRes);
	ElementPtr rBreaker3 = make_shared<ResistorDP>("rbreak3", 3, 0, breakerRes);
	ElementList circElementsBreakerOn;
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
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::DynPhasor);
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
	shared_ptr<SynchronGeneratorDP> genPtr = std::dynamic_pointer_cast<SynchronGeneratorDP>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

	// Calculate initial values for circuit at generator connection point
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);

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

	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::SimpSynGenUnitTestThreePhaseFault() {
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

	Real Ka = 20;
	Real Ta = 0.2;
	Real Ke = 1;
	Real Te = 0.314;
	Real Kf = 0.063;
	Real Tf = 0.35;
	Real Tr = 1;
	Real mVref = 1;

	// Declare circuit components
	ElementPtr gen = make_shared<SimplifiedVBR>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, false);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorEMT>("r1", 1, 0, loadRes);
	ElementPtr r2 = make_shared<ResistorEMT>("r2", 2, 0, loadRes);
	ElementPtr r3 = make_shared<ResistorEMT>("r3", 3, 0, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	//Real breakerRes = 0.001;
	Real breakerRes = 103.78378;
	ElementPtr rBreaker1 = make_shared<ResistorEMT>("rbreak1", 1, 0, breakerRes);
	ElementPtr rBreaker2 = make_shared<ResistorEMT>("rbreak2", 2, 0, breakerRes);
	ElementPtr rBreaker3 = make_shared<ResistorEMT>("rbreak3", 3, 0, breakerRes);
	ElementList circElementsBreakerOn;
	circElementsBreakerOn.push_back(gen);
	circElementsBreakerOn.push_back(rBreaker1);
	circElementsBreakerOn.push_back(rBreaker2);
	circElementsBreakerOn.push_back(rBreaker3);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 15; dt = 0.0001; t = 0;
	Int downSampling = 1;
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
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
	shared_ptr<SimplifiedVBR> genPtr = std::dynamic_pointer_cast<SimplifiedVBR>(gen);
	genPtr->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);
	genPtr->AddExciter(Ta, Ka, Te, Ke, Tf, Kf, Tr, Lmd, Rfd);

	// Calculate initial values for circuit at generator connection point
	Real initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	Real initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	Real initPowerFactor = acos(initActivePower / initApparentPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;
	newSim.setSwitchTime(0.1, 1);
	//newSim.setSwitchTime(2.1, 0);

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
}


void DPsim::SynGenUnitTestVBR() {

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

	ElementPtr gen = make_shared<VoltageBehindReactanceEMT>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H, true);

	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorEMT>("r1", 1, 0, loadRes);
	ElementPtr r2 = make_shared<ResistorEMT>("r2", 2, 0, loadRes);
	ElementPtr r3 = make_shared<ResistorEMT>("r3", 3, 0, loadRes);

	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	ElementPtr rBreaker1 = make_shared<ResistorEMT>("rbreak1", 1, 0, breakerRes);
	ElementPtr rBreaker2 = make_shared<ResistorEMT>("rbreak2", 2, 0, breakerRes);
	ElementPtr rBreaker3 = make_shared<ResistorEMT>("rbreak3", 3, 0, breakerRes);
	ElementList circElementsBreakerOn;
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
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::EMT);
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
	shared_ptr<VoltageBehindReactanceEMT> genPtr = std::dynamic_pointer_cast<VoltageBehindReactanceEMT>(gen);
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
}

void DPsim::SynGenUnitTestVBRDP() {

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
	ElementPtr gen = make_shared<VoltageBehindReactanceDP>("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	Real loadRes = 1037.8378;
	ElementPtr r1 = make_shared<ResistorDP>("r1", 1, 0, loadRes);
	ElementPtr r2 = make_shared<ResistorDP>("r2", 2, 0, loadRes);
	ElementPtr r3 = make_shared<ResistorDP>("r3", 3, 0, loadRes);
	ElementList circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	Real breakerRes = 0.001;
	ElementPtr rBreaker1 = make_shared<ResistorDP>("rbreak1", 1, 0, breakerRes);
	ElementPtr rBreaker2 = make_shared<ResistorDP>("rbreak2", 2, 0, breakerRes);
	ElementPtr rBreaker3 = make_shared<ResistorDP>("rbreak3", 3, 0, breakerRes);
	ElementList circElementsBreakerOn;
	circElementsBreakerOn.push_back(rBreaker1);
	circElementsBreakerOn.push_back(rBreaker2);
	circElementsBreakerOn.push_back(rBreaker3);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	Real tf, dt, t;
	Real om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.0001; t = 0;
	Int downSampling = 1;
	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::DynPhasor);
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
	shared_ptr<VoltageBehindReactanceDP> genPtr = std::dynamic_pointer_cast<VoltageBehindReactanceDP>(gen);
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
	
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorVBR(vtLog, jLog, gen, newSim.getTime());
		newSim.increaseByTimeStep();
	}

}

//void DPsim::ExciterFirstTest() {
//
//	// Define Object for saving data on a file
//	Logger log("log.txt"),
//		vtLog("data_vt.csv"),
//		jLog("data_j.csv");
//
//
//	// Define machine parameters in per unit
//	Real Ka = 20;
//	Real Ta = 0.2;
//	Real Ke = 1;
//	Real Te = 0.314;
//	Real Kf = 0.063;
//	Real Tf = 0.35;
//	Real Tr = 1;
//	Real mVref = 1;
//
//	// Declare circuit components
//	ElementPtr exc = make_shared<Exciter>(Ta, Ka, Te, Ke, Tf, Kf, Tr, true);
//	ElementList circElements;
//	circElements.push_back(exc);
//
//	// Set up simulation
//	Real tf, dt, t;
//	Real om = 2.0*M_PI*60.0;
//	tf = 10; dt = 0.001; t = 0;
//	Int downSampling = 1;
//	Simulation newSim(circElements, om, dt, tf, log, downSampling, SimulationType::DynPhasor);
//	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);
//	newSim.switchSystemMatrix(0);
//
//	// Initialize generator
//
//	shared_ptr<Exciter> genPtr = std::dynamic_pointer_cast<Exciter>(exc);
//	genPtr->init(mVref);
//
//	std::cout << "A matrix:" << std::endl;
//	std::cout << newSim.getSystemMatrix() << std::endl;
//	std::cout << "vt vector:" << std::endl;
//	std::cout << newSim.getLeftSideVector() << std::endl;
//	std::cout << "j vector:" << std::endl;
//	std::cout << newSim.getRightSideVector() << std::endl;
//
//
//	Real lastLogTime = 0;
//	Real logTimeStep = 0.001;
//
//	while (newSim.getTime() < tf) {
//		std::cout << newSim.getTime() << std::endl;
//		newSim.stepGeneratorVBR(vtLog, jLog, exc, newSim.getTime());
//		newSim.increaseByTimeStep();
//	}
//
//}