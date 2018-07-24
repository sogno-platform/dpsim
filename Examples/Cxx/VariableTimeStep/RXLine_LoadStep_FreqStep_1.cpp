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
using namespace CPS::Components;

static void VarFreqRxLineResLoad_DP(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define system topology
	SystemTopology system0(50, {
		std::make_shared<DP::VoltageSourceFreq>("v_s", 0, DEPRECATEDGND, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime),
		DP::Resistor::make("r_line", 0, 1, 1),
		DP::Inductor::make("l_line", 1, 2, 0.2)});

	SystemTopology system1 = system0;
	SystemTopology system2 = system0;
	system1.mComponents.push_back(DP::Resistor::make("r_load", 2, DEPRECATEDGND, 100));
	system2.mComponents.push_back(DP::Resistor::make("r_load", 2, DEPRECATEDGND, 50));
	
	// Define simulation scenario
	String simName = "DP_RXLine_LoadStep_FreqStep_1_" + std::to_string(timeStep);

	Simulation sim(simName, system1, timeStep, finalTime);
	sim.addSystemTopology(system2);
	sim.setSwitchTime(loadStep, 1);

	sim.run();
}

static void VarFreqRxLineResLoad_EMT(Real timeStep, Real finalTime, Real freqStep, Real loadStep, Real rampTime) {
	// Define system topology
	SystemTopology system0(50, {
		std::make_shared<EMT::VoltageSourceFreq>("v_s", 0, DEPRECATEDGND, 1000, 0, 1, 2 * PI*-5, freqStep, rampTime),
		EMT::Resistor::make("r_line", 0, 1, 1),
		EMT::Inductor::make("l_line", 1, 2, 0.2)});

	SystemTopology system1 = system0;
	SystemTopology system2 = system0;
	system1.mComponents.push_back(EMT::Resistor::make("r_load", 2, DEPRECATEDGND, 100));
	system2.mComponents.push_back(EMT::Resistor::make("r_load", 2, DEPRECATEDGND, 50));
	
	// Define simulation scenario
	String simName = "EMT_RXLine_LoadStep_FreqStep_1_" + std::to_string(timeStep);	

	Simulation sim(simName, system1, timeStep, finalTime, Domain::EMT);
	sim.addSystemTopology(system2);
	sim.setSwitchTime(loadStep, 1);

	sim.run();
}

int main(int argc, char* argv[]) {
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
