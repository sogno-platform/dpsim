/** Reference Circuits
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include "Simulation.h"
#include "Utilities.h"

using namespace DPsim;

static void VarFreqRxLineResLoad_DP(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime)
{
	// Define simulation scenario
	Real omega = 2.0*M_PI*50.0;
	std::ostringstream fileName;
	fileName << "DpEmtVarFreqStudy_" << timeStep;
	Components::Base::List circElements0, circElements1, circElements2;
	circElements0.push_back(std::make_shared<Components::DP::VoltageSourceFreq>("v_s", 1, 0, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime));
	circElements0.push_back(std::make_shared<Components::DP::Resistor>("r_line", 1, 2, 1));
	circElements0.push_back(std::make_shared<Components::DP::Inductor>("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(std::make_shared<Components::DP::Resistor>("r_load", 3, 0, 100));
	circElements2.push_back(std::make_shared<Components::DP::Resistor>("r_load", 3, 0, 50));

	// Define log names
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements1, omega, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);
	std::cout << "Start simulation." << std::endl;

	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}

	std::cout << "Simulation finished." << std::endl;
}

static void VarFreqRxLineResLoad_EMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime)
{
	// Define simulation scenario
	Real omega = 2.0*M_PI*50.0;
	std::ostringstream fileName;
	fileName << "RXLineResLoadEMT_" << timeStep;
	Components::Base::List circElements0, circElements1, circElements2;
	circElements0.push_back(std::make_shared<Components::DP::VoltageSourceFreq>("v_s", 1, 0, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime));
	circElements0.push_back(std::make_shared<Components::EMT::Resistor>("r_line", 1, 2, 1));
	circElements0.push_back(std::make_shared<Components::EMT::Inductor>("l_line", 2, 3, 0.2));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(std::make_shared<Components::EMT::Resistor>("r_load", 3, 0, 100));
	circElements2.push_back(std::make_shared<Components::EMT::Resistor>("r_load", 3, 0, 50));

	// Define log names
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation and start main simulation loop
	Simulation newSim(circElements1, omega, timeStep, finalTime, log, SimulationType::EMT);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);
	std::cout << "Start simulation." << std::endl;

	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}

	std::cout << "Simulation finished." << std::endl;
}

int main(int argc, char* argv[])
{
	Real timeStep = 0.0;
	Real finalTime = 0.6;
	Real freqStep = 0.4;
	Real loadStep = 0.2;
	Real rampTime = 0;

	timeStep = 0.00005;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.001;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.005;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.01;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.015;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.02;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.025;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.03;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.035;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.04;
	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	return 0;
}
