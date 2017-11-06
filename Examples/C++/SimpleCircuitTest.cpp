/** Simple Circuit Test
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

#include "SimpleCircuitTest.h"
#include "Simulation.h"
#include "Utilities.h"

using namespace DPsim;

void DPsim::RXLineResLoad() {
	Real timeStep = 0.001;

	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "RXLineResLoad_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceRes("v_s", 1, 0, Complex(10000, 0), 1));
	circElements0.push_back(new ResistorDP("r_line", 1, 2, 1));
	circElements0.push_back(new InductorDP("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new ResistorDP("r_load", 3, 0, 1000));
	circElements2.push_back(new ResistorDP("r_load", 3, 0, 800));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*M_PI*50.0, timeStep, 0.3, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(0.1, 1);

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
	delete circElements1[3];
	delete circElements2[3];
}

void DPsim::VarFreqRXLineResLoad(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoad_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreq("v_s", 1, 0, 1000, 0, 1, 2*PI*-5, freqStep, rampTime));
	circElements0.push_back(new ResistorDP("r_line", 1, 2, 1));
	circElements0.push_back(new InductorDP("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new ResistorDP("r_load", 3, 0, 100));
	circElements2.push_back(new ResistorDP("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

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
	delete circElements1[3];
	delete circElements2[3];
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
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResEMT("v_s", 1, 0, Complex(10000, 0), 1));
	circElements0.push_back(new LinearResistorEMT("r_line", 1, 2, 1));
	circElements0.push_back(new InductorEMT("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistorEMT("r_load", 3, 0, 1000));
	circElements2.push_back(new LinearResistorEMT("r_load", 3, 0, 800));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, 0.3, log, SimulationType::EMT);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(0.1, 1);

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
	delete circElements1[3];
	delete circElements2[3];
}

void DPsim::VarFreqRXLineResLoadEMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoadEMT_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreqEMT("v_s", 1, 0, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime));
	circElements0.push_back(new LinearResistorEMT("r_line", 1, 2, 1));
	circElements0.push_back(new InductorEMT("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistorEMT("r_load", 3, 0, 100));
	circElements2.push_back(new LinearResistorEMT("r_load", 3, 0, 50));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

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
	delete circElements1[3];
	delete circElements2[3];
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
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreq("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(new ResistorDP("r_line", 1, 2, 1));
	circElements0.push_back(new InductorDP("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new ResistorDP("r_load", 3, 0, 10));
	circElements2.push_back(new ResistorDP("r_load", 3, 0, 5));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

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
	delete circElements1[3];
	delete circElements2[3];
}

void DPsim::VarFreqRXLineResLoadEMT_NZ_Paper(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define Object for saving data on a file
	std::ostringstream fileName;
	fileName << "VarFreqRXLineResLoadEMT_" << timeStep;
	Logger log("Logs/Log_" + fileName.str() + ".log"),
		leftVectorLog("Logs/LeftVectorLog_" + fileName.str() + ".csv"),
		rightVectorLog("Logs/RightVectorLog_" + fileName.str() + ".csv");

	// Declare circuit components
	std::vector<BaseComponent*> circElements0, circElements1, circElements2;
	circElements0.push_back(new VoltSourceResFreqEMT("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(new LinearResistorEMT("r_line", 1, 2, 1));
	circElements0.push_back(new InductorEMT("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(new LinearResistorEMT("r_load", 3, 0, 10));
	circElements2.push_back(new LinearResistorEMT("r_load", 3, 0, 8));

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);

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
	delete circElements1[3];
	delete circElements2[3];
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

#ifdef __linux__
void DPsim::RTExample() {
	std::vector<BaseComponent*> comps;
	Logger log;

	comps.push_back(new VoltSourceRes("v_s", 1, 0, Complex(10000, 0), 1));
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
