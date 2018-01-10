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
	String simName = "DpEmtVarFreqStudy_NZ_Paper_" + std::to_string(timeStep);
	Components::Base::List comps0, comps1, comps2;
	comps0.push_back(std::make_shared<Components::DP::VoltageSourceFreq>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	comps0.push_back(std::make_shared<Components::DP::Resistor>("r_line", 1, 2, 1));
	comps0.push_back(std::make_shared<Components::DP::Inductor>("l_line", 2, 3, 1));
	comps1 = comps0;
	comps2 = comps0;
	comps1.push_back(std::make_shared<Components::DP::Resistor>("r_load", 3, 0, 10));
	comps2.push_back(std::make_shared<Components::DP::Resistor>("r_load", 3, 0, 5));

	// Set up simulation
	Simulation newSim(simName, comps1, 2.0*PI*50.0, timeStep, finalTime);
	newSim.addSystemTopology(comps2);
	newSim.setSwitchTime(loadStep, 1);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step()) {
		newSim.increaseByTimeStep();
	}
	std::cout << "Simulation finished." << std::endl;
}

static void VarFreqRxLineResLoad_NZ_Paper_EMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define simulation scenario
	String simName = "DpEmtVarFreqStudy_NZ_Paper_EMT" + std::to_string(timeStep);
	Components::Base::List comps0, comps1, comps2;
	comps0.push_back(std::make_shared<Components::EMT::VoltageSourceFreq>("v_s", 1, 0, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime));
	comps0.push_back(std::make_shared<Components::EMT::Resistor>("r_line", 1, 2, 1));
	comps0.push_back(std::make_shared<Components::EMT::Inductor>("l_line", 2, 3, 1));
	comps1 = comps0;
	comps2 = comps0;
	comps1.push_back(std::make_shared<Components::EMT::Resistor>("r_load", 3, 0, 10));
	comps2.push_back(std::make_shared<Components::EMT::Resistor>("r_load", 3, 0, 8));

	// Set up simulation
	Simulation newSim(simName, comps1, 2.0*PI*50.0, timeStep, finalTime, Logger::Level::INFO, SimulationType::EMT);

	// Main Simulation Loop
	std::cout << "Start simulation." << std::endl;
	while (newSim.step()) {
		newSim.increaseByTimeStep();
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
