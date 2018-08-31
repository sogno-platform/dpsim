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


#include <DPsim.h>

using namespace DPsim;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Define system topology
	SystemTopology system0(50, {
		VoltageSourceNorton::make("v_s", 0, DEPRECATEDGND, Complex(10000, 0), 1),
		Resistor::make("r_line", 0, 1, 1),
		Inductor::make("l_line", 1, 2, 1)});

	SystemTopology system1 = system0;
	SystemTopology system2 = system0;
	system1.mComponents.push_back(Resistor::make("r_load", 3, 0, 1000));
	system2.mComponents.push_back(Resistor::make("r_load", 3, 0, 800));

	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 0.3;
	String simName = "DP_ResVC_RxLine1_" + std::to_string(timeStep);

	Simulation sim(simName, system1, timeStep, finalTime);
	sim.addSystemTopology(system2);
	sim.setSwitchTime(0.1, 1);

	sim.run();

	return 0;
}
