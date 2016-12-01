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
	BaseComponent* gen = new SynchronousGenerator("gen", 1, 2, 3, Rs, Ll, Lmd, Lmq, Rf, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, J, P);
	BaseComponent* r1 = new LinearResistor("r1", 0, 1, 733.8615);
	BaseComponent* r2 = new LinearResistor("r2", 0, 2, 733.8615);
	BaseComponent* r3 = new LinearResistor("r3", 0, 3, 733.8615);

	std::vector<BaseComponent*> circElements;
	circElements.push_back(gen);
	circElements.push_back(r1);
	circElements.push_back(r2);
	circElements.push_back(r3);

	double tf, dt, t;
	double om = 2.0*M_PI*50.0;

	Simulation newSim(circElements, om, dt, tf, log);
}
