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

static void VarFreqRxLineResLoad_NZ_Paper_DP(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime)
{
	// Define simulation scenario
	std::ostringstream fileName;
	fileName << "DpEmtVarFreqStudy_NZ_Paper_" << timeStep;
	Component::Base::List circElements0, circElements1, circElements2;
	circElements0.push_back(std::make_shared<Component::DP::VoltageSourceFreq>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(std::make_shared<Component::DP::Resistor>("r_line", 1, 2, 1));
	circElements0.push_back(std::make_shared<Component::DP::Inductor>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(std::make_shared<Component::DP::Resistor>("r_load", 3, 0, 10));
	circElements2.push_back(std::make_shared<Component::DP::Resistor>("r_load", 3, 0, 5));

	// Define log names
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log);
	newSim.addSystemTopology(circElements2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;

	while (newSim.step(leftVectorLog, rightVectorLog)) {
		newSim.increaseByTimeStep();
		updateProgressBar(newSim.getTime(), newSim.getFinalTime());
	}

	std::cout << "Simulation finished." << std::endl;
}

static void VarFreqRxLineResLoad_NZ_Paper_EMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define simulation scenario
	std::ostringstream fileName;
	fileName << "DpEmtVarFreqStudy_NZ_Paper_EMT" << timeStep;
	Component::Base::List circElements0, circElements1, circElements2;
	circElements0.push_back(std::make_shared<Component::EMT::VoltageSourceFreq>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	circElements0.push_back(std::make_shared<Component::EMT::Resistor>("r_line", 1, 2, 1));
	circElements0.push_back(std::make_shared<Component::EMT::Inductor>("l_line", 2, 3, 1));
	circElements1 = circElements0;
	circElements2 = circElements0;
	circElements1.push_back(std::make_shared<Component::EMT::Resistor>("r_load", 3, 0, 10));
	circElements2.push_back(std::make_shared<Component::EMT::Resistor>("r_load", 3, 0, 8));

	// Define log names
	Logger log("Logs/" + fileName.str() + ".log");
	Logger leftVectorLog("Logs/LeftVector_" + fileName.str() + ".csv");
	Logger rightVectorLog("Logs/RightVector_" + fileName.str() + ".csv");

	// Set up simulation
	Simulation newSim(circElements1, 2.0*PI*50.0, timeStep, finalTime, log, SimulationType::EMT);

	// Main Simulation Loop
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
	Real finalTime = 2;
	Real freqStep = 1;
	Real loadStep = 10;
	Real rampTime = 0.2;

	timeStep = 0.00005;
	VarFreqRxLineResLoad_NZ_Paper_EMT(timeStep, finalTime, freqStep, loadStep, rampTime);

	timeStep = 0.001;
	VarFreqRxLineResLoad_NZ_Paper_DP(timeStep, finalTime, freqStep, loadStep, rampTime);

	return 0;
}
