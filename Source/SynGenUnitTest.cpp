#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "Simulation.h"
#include "Components.h"
#include "Logger.h"


void SysGenUnitTest() {
	
	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

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
	BaseComponent* gen = new SynchronousGenerator("gen", 1, 2, 3, 
		SynchGenStateType::perUnit, nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		SynchGenParamType::perUnit, Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H);
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
	tf = 0.1; dt = 0.000050; t = 0;
	Simulation newSim(circElements, om, dt, tf, log);

	// Initialize generator
	double initActivePower = 555e3;
	double initReactivePower = 0;
	double initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
	double initVoltAngle = -DPS_PI / 2;
	((SynchronousGenerator*)newSim.elements[0])->Init(newSim.A, newSim.j, 0, om, dt,
		initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	// Calculate initial values for circuit at generator connection point
	double initApparentPower = sqrt(pow(initActivePower, 2) + pow(initReactivePower, 2));
	double initTerminalCurr = initApparentPower / (3 * initTerminalVolt)* sqrt(2);	
	double initPowerFactor = acos(initActivePower / initApparentPower);
	newSim.vt(0, 0) = initTerminalVolt * cos(initVoltAngle);
	newSim.vt(1, 0) = initTerminalVolt * cos(initVoltAngle - 2 * M_PI / 3);
	newSim.vt(2, 0) = initTerminalVolt * cos(initVoltAngle + 2 * M_PI / 3);
	newSim.j(0, 0) = initTerminalCurr * cos(initVoltAngle + initPowerFactor);
	newSim.j(1, 0) = initTerminalCurr * cos(initVoltAngle + initPowerFactor - 2 * M_PI / 3);
	newSim.j(2, 0) = initTerminalCurr * cos(initVoltAngle + initPowerFactor + 2 * M_PI / 3);

	std::cout << "A matrix:" << std::endl;
	std::cout << newSim.A << std::endl;
	std::cout << "vt vector:" << std::endl;
	std::cout << newSim.vt << std::endl;
	std::cout << "j vector:" << std::endl;
	std::cout << newSim.j << std::endl;

	// Main Simulation Loop
	while (newSim.t < tf)
	{
		std::cout << newSim.t << std::endl;
		//std::cout << "j vector:" << std::endl;
		//std::cout << newSim.j << std::endl;
		//std::cout << "vt vector:" << std::endl;
		//std::cout << newSim.vt << std::endl;

		// Set to zero because all components will add their contribution for the current time step to the current value
		newSim.j.setZero();

		// Execute step for all circuit components
		for (std::vector<BaseComponent*>::iterator it = newSim.elements.begin(); it != newSim.elements.end(); ++it) {
			(*it)->Step(newSim.A, newSim.j, newSim.compOffset, om, dt, newSim.t);
		}
		
		// Individual step function for generator
		double fieldVoltage = 7.0821;
		double mechPower = 5.5558e5;
		((SynchronousGenerator*)newSim.elements[0])->Step(newSim.A, newSim.j, 0, om, dt, newSim.t, fieldVoltage, mechPower);

		// Solve circuit for vector j with generator output current
		newSim.vt = newSim.luFactored.solve(newSim.j);

		// Execute PostStep for all components, generator states are recalculated based on new terminal voltage
		for (std::vector<BaseComponent*>::iterator it = newSim.elements.begin(); it != newSim.elements.end(); ++it) {
			(*it)->PostStep(newSim.A, newSim.j, newSim.vt, newSim.compOffset, om, dt, newSim.t);
		}		

		// Save simulation step data
		vtLog.Log() << newSim.GetVoltageDataLine().str();
		jLog.Log() << newSim.GetCurrentDataLine().str();
		
		// timestep
		newSim.t += dt;
	}

	// Write simulation data to file
	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");
	
	std::cout << "Simulation finished." << std::endl;
}
