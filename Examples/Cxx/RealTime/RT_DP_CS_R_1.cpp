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
using namespace CPS::Components::DP;

int main(int argc, char* argv[]) {
	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto cs = CurrentSource::make("cs", Node::List{GND, n1}, Complex(10, 0), Logger::Level::DEBUG);
	auto r1 = Resistor::make("r_1", Node::List{GND, n1}, 1);

	// Define system topology
	auto sys = SystemTopology(50, Node::List{GND, n1}, ComponentBase::List{cs, r1});

	// Define simulation scenario
	Real timeStep = 0.001;
	Real finalTime = 5;
	String simName = "RT_DP_CS_R_1";

	RealTimeSimulation sim(simName, sys, timeStep, finalTime);
	sim.run();

	return 0;
}
