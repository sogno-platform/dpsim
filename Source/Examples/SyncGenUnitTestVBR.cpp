#include "SyncGenUnitTestVBR.h"

#include "../Simulation.h"
#include "../Utilities.h"

using namespace DPsim;

void DPsim::SynGenUnitTestVBR() {

	// Define Object for saving data on a file
	Logger log("Logs/logVBR.txt"),
		synGenLogVolt("Logs/data_synGen_volt.csv"),
		synGenLogCurr("Logs/data_synGen_curr.csv");

	// Define machine parameters - stator referred
	double nomPower = 555e6;
	double nomPhPhVoltRMS = 24e3;
	double nomFreq = 60;
	double nomFieldCurr = 1300;
	int poleNum = 2;
	double J = 28897.6459179918;
	double H = 3.7;

	double Rs = 0.0031;
	double Ll = 0.0004129;
	double Lmd = 0.0046;
	double Lmq = 0.0044;
	double Rfd = 0.0006226;
	double Llfd = 0.0004538;
	double Rkd = 0.0295;
	double Llkd = 0.0005;
	double Rkq1 = 0.0064;
	double Llkq1 = 0.002;
	double Rkq2 = 0.0246;
	double Llkq2 = 0.0003;

	// Declare circuit components
	BaseComponent* gen = new VoltageBehindReactanceEMT("gen", 1, 2, 3,
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);

	// Set up simulation
	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.01; dt = 0.000001; t = 0;
	Simulation newSim(circElements, om, dt, tf, log, SimulationType::EMT);
 
	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((VoltageBehindReactanceEMT*)gen)->init(om, dt, initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	//std::cout << "A matrix:" << std::endl;
	//std::cout << newSim.getSystemMatrix() << std::endl;
	//std::cout << "vt vector:" << std::endl;
	//std::cout << newSim.getLeftSideVector() << std::endl;
	//std::cout << "j vector:" << std::endl;
	//std::cout << newSim.getRightSideVector() << std::endl;

	Real fieldVoltage = 7.0821;
	Real mechPower = 5.5558e5;
	Real lastLogTime = 0;
	Real logTimeStep = 0.00005;

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;
		newSim.stepGeneratorVBR(log, gen, synGenLogVolt, synGenLogCurr, fieldVoltage, mechPower, logTimeStep, lastLogTime, newSim.getTime());
		newSim.increaseByTimeStep();
	}

	std::cout << "Simulation finished." << std::endl;
	for (auto elem : circElements)
		delete elem;
}

