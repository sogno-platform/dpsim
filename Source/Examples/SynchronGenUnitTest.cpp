#include "SynchronGenUnitTest.h"

#include "../Simulation.h"
#include "../Utilities.h"

using namespace DPsim;

void DPsim::SynGenUnitTestBalancedResLoad() {
	
	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv"),
		synGenLogVolt("data_synGen_flux.csv"),
		synGenLogCurr("data_synGen_volt.csv"),
		synGenLogFlux("data_synGen_curr.csv");

	// Define machine parameters - stator referred
	//double nomPower = 555e6;
	//double nomPhPhVoltRMS = 24e3;
	//double nomFreq = 60;
	//double nomFieldCurr = 1300;
	//int P = 2;
	//double J = 28897.6459179918;
	//double Rs = 0.0031;
	//double Ll = 0.0004129;
	//double Lmd = 0.0046;
	//double Lmq = 0.0044;
	//double Rf = 0.0006226;
	//double Llfd = 0.0004538;
	//double Rkd = 0.0295;
	//double Llkd = 0.0005;
	//double Rkq1 = 0.0064;
	//double Llkq1 = 0.002;
	//double Rkq2 = 0.0246;
	//double Llkq2 = 0.0003;

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	double Rkq2 = 0.0237;
	double Llkq2 = 0.125;

	// Declare circuit components
	BaseComponent* gen = new SynchronGeneratorEMT("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	double loadRes = 1037.8378;
	BaseComponent* r1 = new LinearResistorEMT("r1", 0, 1, loadRes);
	BaseComponent* r2 = new LinearResistorEMT("r2", 0, 2, loadRes);
	BaseComponent* r3 = new LinearResistorEMT("r3", 0, 3, loadRes);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.1; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT);
	newSim.setNumericalMethod(NumericalMethod::Euler);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronGeneratorEMT*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	// Calculate initial values for circuit at generator connection point
	//double initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	//double initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);	
	//double initPowerFactor = acos(initActivePower / initApparentPower);
	//double initVolt1 = initTerminalVolt * cos(initVoltAngle);
	//double initVolt2 = initTerminalVolt * cos(initVoltAngle - 2 * M_PI / 3);
	//double initVolt3 = initTerminalVolt * cos(initVoltAngle + 2 * M_PI / 3);
	//double initCurrent1 = initTerminalCurr * cos(initVoltAngle + initPowerFactor);
	//double initCurrent2 = initTerminalCurr * cos(initVoltAngle + initPowerFactor - 2 * M_PI / 3);
	//double initCurrent3 = initTerminalCurr * cos(initVoltAngle + initPowerFactor + 2 * M_PI / 3);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;		
		newSim.stepGeneratorTest(log, vtLog, jLog, gen, synGenLogFlux, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();
	}
	
	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}

void DPsim::SynGenUnitTestPhaseToPhaseFault() {
	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv"),
		synGenLogVolt("data_synGen_flux.csv"),
		synGenLogCurr("data_synGen_volt.csv"),
		synGenLogFlux("data_synGen_curr.csv");

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	double Rkq2 = 0.0237;
	double Llkq2 = 0.125;

	// Declare circuit components
	BaseComponent* gen = new SynchronGeneratorEMT("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	double loadRes = 1037.8378;
	BaseComponent* r1 = new LinearResistorEMT("r1", 0, 1, loadRes);
	BaseComponent* r2 = new LinearResistorEMT("r2", 0, 2, loadRes);
	BaseComponent* r3 = new LinearResistorEMT("r3", 0, 3, loadRes);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	double breakerRes = 0.01;
	BaseComponent* rBreaker = new LinearResistorEMT("rbreak", 1, 2, breakerRes);

	std::vector<BaseComponent*> circElementsBreakerOn;
	circElementsBreakerOn.push_back(rBreaker);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.2; dt = 0.00005; t = 0;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT);
	newSim.addSystemTopology(circElementsBreakerOn);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronGeneratorEMT*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	// Calculate initial values for circuit at generator connection point
	double initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	double initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	double initPowerFactor = acos(initActivePower / initApparentPower);
	
	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;
	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(log, vtLog, jLog, gen, synGenLogFlux, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
	delete rBreaker;
}

void DPsim::SynGenUnitTestThreePhaseFault() {
	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv"),
		synGenLogVolt("data_synGen_flux.csv"),
		synGenLogCurr("data_synGen_volt.csv"),
		synGenLogFlux("data_synGen_curr.csv");

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	double Rkq2 = 0.0237;
	double Llkq2 = 0.125;

	// Declare circuit components
	BaseComponent* gen = new SynchronGeneratorEMT("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	double loadRes = 1037.8378;
	BaseComponent* r1 = new LinearResistorEMT("r1", 1, 0, loadRes);
	BaseComponent* r2 = new LinearResistorEMT("r2", 2, 0, loadRes);
	BaseComponent* r3 = new LinearResistorEMT("r3", 3, 0, loadRes);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Declare circuit components for resistance change
	double breakerRes = 0.001;
	BaseComponent* rBreaker1 = new LinearResistorEMT("rbreak1", 1, 0, breakerRes);
	BaseComponent* rBreaker2 = new LinearResistorEMT("rbreak2", 2, 0, breakerRes);
	BaseComponent* rBreaker3 = new LinearResistorEMT("rbreak3", 3, 0, breakerRes);
	std::vector<BaseComponent*> circElementsBreakerOn;
	circElementsBreakerOn.push_back(rBreaker1);
	circElementsBreakerOn.push_back(rBreaker2);
	circElementsBreakerOn.push_back(rBreaker3);
	circElementsBreakerOn.push_back(r1);
	circElementsBreakerOn.push_back(r2);
	circElementsBreakerOn.push_back(r3);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.3; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_current);
	newSim.addSystemTopology(circElementsBreakerOn);
	newSim.switchSystemMatrix(0);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronGeneratorEMT*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	// Calculate initial values for circuit at generator connection point
	double initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	double initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	double initPowerFactor = acos(initActivePower / initApparentPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;
	
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;
	newSim.setSwitchTime(0.1, 1);
	newSim.setSwitchTime(0.2, 0);

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(log, vtLog, jLog, gen, synGenLogFlux, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();		
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
	delete rBreaker1;
	delete rBreaker2;
	delete rBreaker3;
}

void DPsim::SynGenDPUnitTestBalancedResLoad() {
	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv"),
		synGenLogVolt("data_synGen_flux.csv"),
		synGenLogCurr("data_synGen_volt.csv"),
		synGenLogFlux("data_synGen_curr.csv");

	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	double Rkq2 = 0.0237;
	double Llkq2 = 0.125;

	// Declare circuit components
	BaseComponent* gen = new SynchronGenerator("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	double loadRes = 1037.8378;
	BaseComponent* r1 = new LinearResistor("r1", 0, 1, loadRes);
	BaseComponent* r2 = new LinearResistor("r2", 0, 2, loadRes);
	BaseComponent* r3 = new LinearResistor("r3", 0, 3, loadRes);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.01; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log);
	newSim.setNumericalMethod(NumericalMethod::Euler);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronGenerator*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	// Calculate initial values for circuit at generator connection point
	double initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	double initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);
	double initPowerFactor = acos(initActivePower / initApparentPower);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;
	
	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf) {
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorTest(log, vtLog, jLog, gen, synGenLogFlux, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();		
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}

void DPsim::SynGenUnitTestdqBalancedResLoad() {

	// Define Object for saving data on a file
	Logger log("log.txt"),
		vtLog("data_vt.csv"),
		jLog("data_j.csv"),
		synGenLogVolt("data_synGen_flux.csv"),
		synGenLogCurr("data_synGen_volt.csv"),
		synGenLogFlux("data_synGen_curr.csv");


	// Define machine parameters in per unit
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 2.8898e+04;
	double H = 3.7;

	double Rs = 0.003;
	double Ll = 0.15;
	double Lmd = 1.6599;
	double Lmd0 = 1.6599;
	double Lmq = 1.61;
	double Lmq0 = 1.61;
	double Rfd = 0.0006;
	double Llfd = 0.1648;
	double Rkd = 0.0284;
	double Llkd = 0.1713;
	double Rkq1 = 0.0062;
	double Llkq1 = 0.7252;
	double Rkq2 = 0.0237;
	double Llkq2 = 0.125;

	// Declare circuit components
	BaseComponent* gen = new SynchronGeneratorEMTdq("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
	double loadRes = 1037.8378;
	//BaseComponent* r1 = new LinearResistorEMT("r1", 0, 1, loadRes);
	//BaseComponent* r2 = new LinearResistorEMT("r2", 0, 2, loadRes);
	//BaseComponent* r3 = new LinearResistorEMT("r3", 0, 3, loadRes);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	//circElements.push_back(r1);
	//circElements.push_back(r2);
	//circElements.push_back(r3);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.1; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT);
	newSim.setNumericalMethod(NumericalMethod::Trapezoidal_flux);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronGeneratorEMTdq*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.getSystemMatrix() << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.getLeftSideVector() << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratordq(log, vtLog, jLog, gen, synGenLogFlux, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}
