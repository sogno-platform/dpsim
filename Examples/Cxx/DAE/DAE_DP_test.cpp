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

#include "DAESimulation.h"
#include "DPsim.h"

using namespace DPsim;
using namespace CPS::Components::DP;

int main(int argc, char* argv[])
{
	Real timeStep = 0.00005;

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");

	// Components
	auto vs = VoltageSource::make("v_s", Node::List{GND, n1}, Complex(10000, 0));
	auto rl = Resistor::make("r_line",Node::List{n1, n2}, 1);
	auto ll = Inductor::make("l_line",Node::List{n2, n3}, 1);
	auto rL = Resistor::make("r_load",Node::List{GND, n3}, 1000);

	String simName = "DAE_DP_test" + std::to_string(timeStep);

	auto sys = SystemTopology(50, Node::List{GND, n1, n2, n3}, Component::List{vs, rl, ll, rL});
	auto sim = DAESimulation(simName, sys, timeStep, 1.0);

	sim.run();

	return 0;
}
