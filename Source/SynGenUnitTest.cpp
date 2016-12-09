#include <iostream>
#include <string>
#include "MathLibrary.h"
#include "Simulation.h"
#include "Components.h"
#include "Logger.h"


void SysGenUnitTest() {
	
	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

	double Rs = 0.0031;
	double Ll = 0.0004129;
	double Lmd = 0.0046;
	double Lmq = 0.0044;
	double Rf = 0.0006226;
	double Llfd = 0.0004538;
	double Rkd = 0.0295;
	double Llkd = 0.0005;
	double Rkq1 = 0.0064;
	double Llkq1 = 0.002;
	double Rkq2 = 0.0246;
	double Llkq2 = 0.0003;
	double J = 0.00028898;
	int P = 2;
	BaseComponent* gen = new SynchronousGenerator("gen", 1, 2, 3, 24e3, Rs, Ll, Lmd, Lmq, Rf, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, J, P);
	BaseComponent* r1 = new LinearResistor("r1", 0, 1, 1036.76071457563);
	BaseComponent* r2 = new LinearResistor("r2", 0, 2, 1036.76071457563);
	BaseComponent* r3 = new LinearResistor("r3", 0, 3, 1036.76071457563);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	double tf, dt, t;
	double om = 2.0*M_PI*60.0;
	tf = 0.01; dt = 0.000001; t = 0;

	Simulation newSim(circElements, om, dt, tf, log);
	newSim.j(0, 0) = 18.9011;
	newSim.j(1, 0) = 18.9011*cos(-120. / 180.*M_PI);
	newSim.j(2, 0) = 18.9011*cos(120. / 180.*M_PI);
	newSim.vt(0, 0) = 24000 / sqrt(3) * sqrt(2);
	newSim.vt(1, 0) = 24000 / sqrt(3)* sqrt(2) * cos(-120. / 180.*M_PI);
	newSim.vt(2, 0) = 24000 / sqrt(3) * sqrt(2) * cos(120. / 180.*M_PI);
	DPSMatrix initVoltages = DPSMatrix::Zero(3, 1);
	DPSMatrix initCurrents = DPSMatrix::Zero(3, 1);
	initVoltages << newSim.vt(0, 0), newSim.vt(1, 0), newSim.vt(2, 0);
	initCurrents << newSim.j(0, 0), newSim.j(1, 0), newSim.j(2, 0);
	((SynchronousGenerator*)newSim.elements[0])->Init(newSim.A, newSim.j, 0, om, dt, initCurrents, initVoltages, 7.0821, 11375, 0);

	// Get Current Simulation Time
	t = newSim.GetTime();
	std::cout << newSim.A << std::endl;

	// Main Simulation Loop
	while (t < tf)
	{
		std::cout << "j vector:" << std::endl;
		std::cout << newSim.j << std::endl;
		std::cout << "vt vector:" << std::endl;
		std::cout << newSim.vt << std::endl;

		newSim.j.setZero();

		for (std::vector<BaseComponent*>::iterator it = newSim.elements.begin(); it != newSim.elements.end(); ++it) {
			(*it)->Step(newSim.A, newSim.j, newSim.compOffset, om, dt, t);
		}

		((SynchronousGenerator*)newSim.elements[0])->Step(newSim.A, newSim.j, 0, om, dt, t, 7.0821, 11375, 555e6);

		newSim.vt = newSim.luFactored.solve(newSim.j);

		for (std::vector<BaseComponent*>::iterator it = newSim.elements.begin(); it != newSim.elements.end(); ++it) {
			(*it)->PostStep(newSim.A, newSim.j, newSim.vt, newSim.compOffset, om, dt, t);
		}

		t += dt;	

		// Save Simulation Step
		vtLog.Log() << newSim.GetVoltageDataLine().str();
		jLog.Log() << newSim.GetCurrentDataLine().str();
		// Get Current Simulation Time
		t = newSim.GetTime();

	}

	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");
}
