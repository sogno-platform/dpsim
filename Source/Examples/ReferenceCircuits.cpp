#include "ReferenceCircuits.h"

#include "../Simulation.h"
#include "../Utilities.h"


using namespace DPsim;

void DPsim::simulationExample1()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExample1_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	circElements0.push_back(new Inductor("l_3", 2, 3, 0.05));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 2));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() <<  std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample1L2()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExample1L2_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	circElements0.push_back(new Inductor("l_3", 2, 3, 0.05));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 2));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample2()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExample2_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new Inductor("l_1", 1, 2, 0.02));
	circElements0.push_back(new Inductor("l_2", 2, 0, 0.1));
	
	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExample3()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExample3_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new VoltSourceRes("v_in", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new Capacitor("c_1", 1, 2, 0.001));
	circElements0.push_back(new Inductor("l_1", 2, 0, 0.001));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleIdealVS()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_in", 1, 2, Complex(10, 0), 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 0, 1));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));
	circElements0.push_back(new LinearResistor("r_3", 2, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleIdealVS2()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS2_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_in", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 2, 1));
	circElements0.push_back(new Capacitor("c_1", 2, 3, 0.001));
	circElements0.push_back(new Inductor("l_1", 3, 0, 0.001));
	circElements0.push_back(new LinearResistor("r_2", 3, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}


void DPsim::simulationExampleIdealVS3()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleIdealVS3_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new LinearResistor("r_1", 1, 2, 1));
	circElements0.push_back(new LinearResistor("r_2", 2, 0, 1));
	circElements0.push_back(new LinearResistor("r_3", 2, 3, 1));
	circElements0.push_back(new LinearResistor("r_4", 3, 0, 1));
	circElements0.push_back(new LinearResistor("r_5", 3, 4, 1));
	circElements0.push_back(new IdealVoltageSource("v_2", 4, 0, Complex(20, 0), 2));




	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new RxLine("Line_1", 1, 2, 3, 0.1, 0.001));
	circElements0.push_back(new LinearResistor("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine2()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine2_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(10, 0), 1));

	circElements0.push_back(new Inductor("l_L", 2, 3, 0.001));
	circElements0.push_back(new LinearResistor("r_L", 1, 2, 0.1));
	circElements0.push_back(new LinearResistor("r_1", 3, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExampleRXLine3()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExampleRXLine3_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(10, 0), 1));
	circElements0.push_back(new RxLine("Line_1", 1, 2, 0.1, 0.001));
	circElements0.push_back(new LinearResistor("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExamplePiLine()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExamplePiLine_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(345, 0), 1));
	circElements0.push_back(new LinearResistor("r1", 1, 2, 5));
	circElements0.push_back(new PiLine("PiLine1", 2, 3, 4, 6.4, 0.186, 0.004));
	circElements0.push_back(new LinearResistor("r_load", 3, 0, 150));



	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}

void DPsim::simulationExamplePiLine2()
{
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "SimulationExamplePiLine2_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	std::vector<BaseComponent*> circElements0;
	circElements0.push_back(new IdealVoltageSource("v_1", 1, 0, Complex(345, 0), 1));
	circElements0.push_back(new LinearResistor("r1", 1, 2, 5));
	circElements0.push_back(new Capacitor("c_1", 2, 0, 0.002));
	circElements0.push_back(new LinearResistor("r_load", 2, 4, 6.4));
	circElements0.push_back(new Inductor("l_1", 4, 3, 0.186));
	circElements0.push_back(new Capacitor("c_2", 3, 0, 0.002));
	circElements0.push_back(new LinearResistor("r_load", 3, 0, 150));



	std::cout << "The contents of circElements0 are:";
	for (std::vector<BaseComponent*>::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(log, leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;

	for (auto elem : circElements0)
		delete elem;
}
