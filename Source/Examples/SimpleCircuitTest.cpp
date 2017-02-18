#include "SimpleCircuitTest.h"

#include "../Simulation.h"
#include "../Utilities.h"

using namespace DPsim;

Real timeStep = 0.02;
Real finalTime = 1;
Real freqStep = 0.6;
Real loadStep = 0.4;

void RXLineResLoad() {
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

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
	Real timeStep = 0.001;
	Simulation newSim(circElements1, 2.0*M_PI*50.0, timeStep, 0.3, log);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(0.1, 1);
			
	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "RXLineResLoad_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
}

void VarFreqRXLineResLoad() {
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreq("v_s", 1, 0, 1000, 0, 1, 2*PI*-10, freqStep));
	circElements0.push_back(new LinearResistor("r_line", 1, 2, 1));
	circElements0.push_back(new Inductor("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistor("r_load", 3, 0, 100));
	circElements2.push_back(new LinearResistor("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoad_" << timeStep << "_" << finalTime;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
}

void RXLineResLoadEMT() {
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

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
	Real timeStep = 0.001;
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, 0.3, log, SimulationType::EMT);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "RXLineResLoadEMT_" << timeStep;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
}

void VarFreqRXLineResLoadEMT() {
	// Define Object for saving data on a file
	Logger log, leftVectorLog, rightVectorLog;

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreqEMT("v_s", 1, 0, 1000, 0, 1, 2 * PI*-10, freqStep));
	circElements0.push_back(new LinearResistorEMT("r_line", 1, 2, 1));
	circElements0.push_back(new InductorEMT("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistorEMT("r_load", 3, 0, 100));
	circElements2.push_back(new LinearResistorEMT("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);
	newSim.CreateSystemMatrix(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	// Write simulation data to file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoadEMT_" << timeStep << "_" << finalTime;
	log.WriteLogToFile("Logs/Log_" + fileName.str() + ".log");
	leftVectorLog.WriteLogToFile("Logs/LeftVectorLog_" + fileName.str() + ".csv");
	rightVectorLog.WriteLogToFile("Logs/RightVectorLog_" + fileName.str() + ".csv");
}