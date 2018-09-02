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
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char* argv[]) {
	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto vs = VoltageSource::make("v_1");
	auto r = Resistor::make("r_1");

	// Topology
	vs->connect({Node::GND, n1});
	r->connect({n1, Node::GND});

	// Parameters
	vs->setParameters(Complex(10, 0));
	r->setParameters(1);

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{vs, r});

	// Define simulation scenario
	Real timeStep = 0.00005;
	Real finalTime = 0.2;
	String simName = "DP_IdealVS_R_1";

	Simulation sim(simName, sys, timeStep, finalTime);
	sim.run();

	return 0;
}
