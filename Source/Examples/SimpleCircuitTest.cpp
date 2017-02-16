#include "SimpleCircuitTest.h"

#include "../Simulation.h"

using namespace DPsim;

void RXLineResLoad() {
	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceRes("v_s", 1, 0, 10000, 0, 1));
	circElements0.push_back(new LinearResistor("r_line", 1, 2, 1));
	circElements0.push_back(new Inductor("l_line", 2, 3, 1));	
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistor("r_load", 3, 0, 1000));	
	circElements2.push_back(new LinearResistor("r_load", 3, 0, 800));

	// Set up simulation
	double tf, dt, t, om; 
	tf = 1; dt = 0.005; t = 0; om = 2.0*M_PI*50.0;
	Simulation newSim(circElements1, om, dt, tf, log);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(0.5, 1);
			
	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;

		newSim.Step(log);	
		
		// Save simulation step data		
		vtLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getLeftSideVector()).str();
		jLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getRightSideVector()).str();		

		// increase timestep
		newSim.mTime += dt;

	}

	// Write simulation data to file
	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");

	std::cout << "Simulation finished." << std::endl;
}

void VarFreqRXLineResLoad() {
	// Define Object for saving data on a file
	Logger log, vtLog, jLog;

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreq("v_s", 1, 0, 10000, 0, 1, 2*PI*5, 0.1));
	circElements0.push_back(new LinearResistor("r_line", 1, 2, 1));
	circElements0.push_back(new Inductor("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistor("r_load", 3, 0, 1000));
	circElements2.push_back(new LinearResistor("r_load", 3, 0, 800));

	// Set up simulation
	double tf, dt, t, om;
	tf = 0.2; dt = 0.00005; t = 0; om = 2.0*M_PI*50.0;
	Simulation newSim(circElements1, om, dt, tf, log);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(0.5, 1);

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;

		newSim.Step(log);

		// Save simulation step data		
		vtLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getLeftSideVector()).str();
		jLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getRightSideVector()).str();

		// increase timestep
		newSim.mTime += dt;

	}

	// Write simulation data to file
	log.WriteLogToFile("log.txt");
	vtLog.WriteLogToFile("data_vt.csv");
	jLog.WriteLogToFile("data_j.csv");

	std::cout << "Simulation finished." << std::endl;
}

void RXLineResLoadEMT() {
	// Define Object for saving data on a file
	Logger log, LeftSideVectorLog, RightSideVectorLog;

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResEMT("v_s", 1, 0, 10000, 0, 1));
	circElements0.push_back(new LinearResistorEMT("r_line", 1, 2, 1));
	circElements0.push_back(new InductorEMT("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistorEMT("r_load", 3, 0, 1000));
	circElements2.push_back(new LinearResistorEMT("r_load", 3, 0, 800));

	// Set up simulation
	double tf, dt, t, om;
	tf = 0.3; dt = 0.005; t = 0; om = 2.0*M_PI*50.0;
	Simulation newSim(circElements1, om, dt, tf, log, SimulationType::EMT);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	while (newSim.getTime() < tf)
	{
		std::cout << newSim.getTime() << std::endl;

		newSim.Step(log);

		// Save simulation step data		
		LeftSideVectorLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getLeftSideVector()).str();
		RightSideVectorLog.Log() << Logger::VectorToDataLine(newSim.getTime(), newSim.getRightSideVector()).str();

		// increase timestep
		newSim.mTime += dt;

	}

	// Write simulation data to file
	log.WriteLogToFile("Log_EMT.txt");
	LeftSideVectorLog.WriteLogToFile("LeftSideVectorLog_EMT.csv");
	RightSideVectorLog.WriteLogToFile("RightSideVectorLog_EMT.csv");

	std::cout << "Simulation finished." << std::endl;
}