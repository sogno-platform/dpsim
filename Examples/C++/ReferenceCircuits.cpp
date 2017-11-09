/** Reference Circuits
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

#include "ReferenceCircuits.h"

#include "Simulation.h"
#include "Utilities.h"
#include "Definitions.h"

using namespace DPsim;

void DPsim::simulationExample1() {	
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	ElementList circElements;
	circElements.push_back(make_shared<VoltSourceRes>("v_in", 1, 0, Complex(10, 0), 1));
	circElements.push_back(make_shared<InductorDP>("l_1", 1, 2, 0.02));
	circElements.push_back(make_shared<InductorDP>("l_2", 2, 0, 0.1));
	circElements.push_back(make_shared<InductorDP>("l_3", 2, 3, 0.05));
	circElements.push_back(make_shared<ResistorDP>("r_2", 3, 0, 2));

	// Define log names
	std::ostringstream fileName;
	fileName << "SimulationExample1_" << timeStep;
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");
		
	// Set up simulation and start main simulation loop
	Simulation newSim(circElements, omega, timeStep, finalTime, log);
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::simulationExample1L2() {
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	ElementList circElements;
	circElements.push_back(make_shared<VoltSourceRes>("v_in", 1, 0, Complex(10, 0), 1));
	circElements.push_back(make_shared<InductorDP>("l_1", 1, 2, 0.02));
	circElements.push_back(make_shared<InductorDP>("l_2", 2, 0, 0.1));
	circElements.push_back(make_shared<InductorDP>("l_3", 2, 3, 0.05));
	circElements.push_back(make_shared<ResistorDP>("r_2", 3, 0, 2));

	// Define log names
	std::ostringstream fileName;
	fileName << "SimulationExample1L2_" << timeStep;
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements, omega, timeStep, finalTime, log);
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;	
}

void DPsim::simulationExample2() {
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	ElementList circElements;
	circElements.push_back(make_shared<VoltSourceRes>("v_in", 1, 0, Complex(10, 0), 1));
	circElements.push_back(make_shared<InductorDP>("l_1", 1, 2, 0.02));
	circElements.push_back(make_shared<InductorDP>("l_2", 2, 0, 0.1));

	// Define log names
	std::ostringstream fileName;
	fileName << "SimulationExample2_" << timeStep;
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements, omega, timeStep, finalTime, log);
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;	
}

void DPsim::simulationExample3() {
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	ElementList circElements;
	circElements.push_back(make_shared<VoltSourceRes>("v_in", 1, 0, Complex(10, 0), 1));
	circElements.push_back(make_shared<Capacitor>("c_1", 1, 2, 0.001));
	circElements.push_back(make_shared<InductorDP>("l_1", 2, 0, 0.001));
	circElements.push_back(make_shared<ResistorDP>("r_2", 2, 0, 1));
	
	// Define log names
	std::ostringstream fileName;
	fileName << "SimulationExample3_" << timeStep;
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements, omega, timeStep, finalTime, log);
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;		
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_in", 1, 2, Complex(10, 0)));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 1, 0, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_2", 2, 0, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_3", 2, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_in", 1, 0, Complex(10, 0)));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 1, 2, 1));
	circElements0.push_back(make_shared<Capacitor>("c_1", 2, 3, 0.001));
	circElements0.push_back(make_shared<InductorDP>("l_1", 3, 0, 0.001));
	circElements0.push_back(make_shared<ResistorDP>("r_2", 3, 0, 1));

	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(10, 0)));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 1, 2, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_2", 2, 0, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_3", 2, 3, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_4", 3, 0, 1));
	circElements0.push_back(make_shared<ResistorDP>("r_5", 3, 4, 1));
	circElements0.push_back(make_shared<IdealVoltageSource>("v_2", 4, 0, Complex(20, 0)));

	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(10, 0)));
	circElements0.push_back(make_shared<RxLine>("Line_1", 1, 2, 0.1, 0.001));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(10, 0)));

	circElements0.push_back(make_shared<InductorDP>("l_L", 2, 3, 0.001));
	circElements0.push_back(make_shared<ResistorDP>("r_L", 1, 2, 0.1));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 3, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(10, 0)));
	circElements0.push_back(make_shared<RxLine>("Line_1", 1, 2, 0.1, 0.001));
	circElements0.push_back(make_shared<ResistorDP>("r_1", 2, 0, 20));


	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(345, 0)));
	circElements0.push_back(make_shared<ResistorDP>("r1", 1, 2, 5));
	circElements0.push_back(make_shared<PiLine>("PiLine1", 2, 3, 4, 6.4, 0.186, 0.004));
	circElements0.push_back(make_shared<ResistorDP>("r_load", 3, 0, 150));

	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
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

	ElementList circElements0;
	circElements0.push_back(make_shared<IdealVoltageSource>("v_1", 1, 0, Complex(345, 0)));
	circElements0.push_back(make_shared<ResistorDP>("r1", 1, 2, 5));
	circElements0.push_back(make_shared<Capacitor>("c_1", 2, 0, 0.002));
	circElements0.push_back(make_shared<ResistorDP>("r_load", 2, 4, 6.4));
	circElements0.push_back(make_shared<InductorDP>("l_1", 4, 3, 0.186));
	circElements0.push_back(make_shared<Capacitor>("c_2", 3, 0, 0.002));
	circElements0.push_back(make_shared<ResistorDP>("r_load", 3, 0, 150));
	
	std::cout << "The contents of circElements0 are:";
	for (ElementList::iterator it = circElements0.begin(); it != circElements0.end(); ++it) {
		std::cout << "Added " << (*it)->getName() << std::endl;
	}
	std::cout << '\n';

	// Set up simulation
	Simulation newSim(circElements0, 2.0*M_PI*50.0, timeStep, 0.3, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::RXLineResLoad() {
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "RXLineResLoad_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
	circElements0.push_back(make_shared<ResistorDP>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorDP>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorDP>("r_load", 3, 0, 1000));
	circElements2.push_back(make_shared<ResistorDP>("r_load", 3, 0, 800));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*M_PI*50.0, timeStep, 0.3, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::VarFreqRXLineResLoad(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoad_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceResFreq>("v_s", 1, 0, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime));
	circElements0.push_back(make_shared<ResistorDP>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorDP>("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorDP>("r_load", 3, 0, 100));
	circElements2.push_back(make_shared<ResistorDP>("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::RXLineResLoadEMT() {
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "RXLineResLoadEMT_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceResEMT>("v_s", 1, 0, Complex(10000, 0), 1));
	circElements0.push_back(make_shared<ResistorEMT>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorEMT>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 1000));
	circElements2.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 800));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, 0.3, log, SimulationType::EMT);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(0.1, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::VarFreqRXLineResLoadEMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoadEMT_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceResFreqEMT>("v_s", 1, 0, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime));
	circElements0.push_back(make_shared<ResistorEMT>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorEMT>("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 100));
	circElements2.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::runDpEmtVarFreqStudy() {
	Real timeStep = 0.0;
	Real finalTime = 0.6;
	Real freqStep = 0.4;
	Real loadStep = 0.2;
	Real rampTime = 0;

	timeStep = 0.00005;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.001;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.005;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.01;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.015;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.02;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.025;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.03;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.035;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.04;
	VarFreqRXLineResLoadEMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRXLineResLoad(timeStep, finalTime, freqStep, loadStep, rampTime);
}

void DPsim::VarFreqRXLineResLoad_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoad_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceResFreq>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(make_shared<ResistorDP>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorDP>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorDP>("r_load", 3, 0, 10));
	circElements2.push_back(make_shared<ResistorDP>("r_load", 3, 0, 5));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::VarFreqRXLineResLoadEMT_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoadEMT_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	ElementList circElements0, circElements1, circElements2;
	circElements0.push_back(make_shared<VoltSourceResFreqEMT>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(make_shared<ResistorEMT>("r_line", 1, 2, 1));
	circElements0.push_back(make_shared<InductorEMT>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 10));
	circElements2.push_back(make_shared<ResistorEMT>("r_load", 3, 0, 8));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog))
	{
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

void DPsim::runDpEmtVarFreqStudy_NZ_Paper() {
	Real timeStep = 0.0;
	Real finalTime = 2;
	Real freqStep = 1;
	Real loadStep = 10;
	Real rampTime = 0.2;

	timeStep = 0.00005;
	VarFreqRXLineResLoadEMT_NZ_Paper(timeStep, finalTime, freqStep, loadStep, rampTime);
	timeStep = 0.001;
	VarFreqRXLineResLoad_NZ_Paper(timeStep, finalTime, freqStep, loadStep, rampTime);
}


void DPsim::simulationExampleTestIdealVoltageSourceEMT() {
	// Define simulation scenario
	Real timeStep = 0.001;
	Real omega = 2.0*M_PI*50.0;
	Real finalTime = 0.3;
	ElementList circElements;
	circElements.push_back(make_shared<IdealVoltageSourceEMT>("v_in", 1, 2, 10));
	circElements.push_back(make_shared<ResistorEMT>("r_1", 1, 0, 5));
	circElements.push_back(make_shared<ResistorEMT>("r_2", 2, 0, 10));
	circElements.push_back(make_shared<ResistorEMT>("r_3", 2, 0, 2));

	// Define log names
	std::ostringstream fileName;
	fileName << "simulationExampleTestIdealVoltageSourceEMT" << timeStep;
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements, omega, timeStep, finalTime, log, SimulationType::EMT);
	std::cout << "Start simulation." << std::endl;
	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}
	std::cout << "Simulation finished." << std::endl;
}

#ifdef __linux__
void DPsim::RTExample() {
	ElementList comps;
	Logger log;

	comps.push_back(make_shared<VoltSourceRes>("v_s", 1, 0, Complex(10000, 0), 1));
	comps.push_back(new LinearResistor("r_line", 1, 2, 1));
	comps.push_back(new Inductor("l_line", 2, 3, 1));
	comps.push_back(new LinearResistor("r_load", 3, 0, 1000));

	// Set up simulation
	Real timeStep = 0.00005;
	Simulation newSim(comps, 2.0*M_PI*50.0, timeStep, 1.0, log);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	newSim.runRT(RTExceptions, false, log, log, log);
	std::cout << "Simulation finished." << std::endl;
	for (auto comp : comps) {
		delete comp;
	}
}
#endif // __linux__