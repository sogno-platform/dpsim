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


#include "DPsim.h"

using namespace DPsim;
using namespace DPsim::Components;

static void VarFreqRxLineResLoad_NZ_Paper_DP(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime)
{
	// Define simulation scenario
	std::ostringstream fileName;
	String simName = "DP_RXLine_LoadStep_FreqStep_2_" + std::to_string(timeStep);

	Component::List comps0 = {
		DP::VoltageSourceFreq::make("v_s", 0, GND, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime),
		DP::Resistor::make("r_line", 0, 1, 1),
		DP::Inductor::make("l_line", 1, 2, 1)
	};

	Component::List comps1 = comps0;
	Component::List comps2 = comps0;
	comps1.push_back(DP::Resistor::make("r_load", 2, GND, 10));
	comps2.push_back(DP::Resistor::make("r_load", 2, GND, 5));

	Simulation sim(simName, comps1, 2.0*PI*50.0, timeStep, finalTime);
	sim.addSystemTopology(comps2);
	sim.setSwitchTime(loadStep, 1);

	sim.run();
}

static void VarFreqRxLineResLoad_NZ_Paper_EMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define simulation scenario
	String simName = "EMT_RXLine_LoadStep_FreqStep_2_" + std::to_string(timeStep);

	Component::List comps0 = {
		EMT::VoltageSourceFreq::make("v_s", 0, GND, 10000, 0, 1, 2 * PI*-1, freqStep, rampTime),
		EMT::Resistor::make("r_line", 0, 1, 1),
		EMT::Inductor::make("l_line", 1, 2, 1)
	};

	Component::List comps1 = comps0;
	Component::List comps2 = comps0;
	comps1.push_back(EMT::Resistor::make("r_load", 2, GND, 10));
	comps2.push_back(EMT::Resistor::make("r_load", 2, GND, 8));

	Simulation sim(simName, comps1, 2.0*PI*50.0, timeStep, finalTime, Logger::Level::INFO, SimulationType::EMT);

	sim.run();
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
