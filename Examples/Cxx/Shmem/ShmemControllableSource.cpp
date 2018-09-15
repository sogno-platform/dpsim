/** Example of shared memory interface
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

int main(int argc, char *argv[]) {

	Interface::Config conf;
	conf.samplelen = 64;
	conf.queuelen = 1024;
	conf.polling = false;

	String in  = "/dpsim10";
	String out = "/dpsim01";

	Real timeStep = 0.001;
	Real finalTime = 10;
	String simName = "ShmemControllableSource";

	Interface intf(out, in, &conf);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto ecs = CurrentSource::make("v_intf", Complex(10, 0));
	auto r1 = Resistor::make("r_1", 1);

	ecs->connect({ Node::GND, n1 });
	r1->connect({ Node::GND, n1 });

	intf.addImport(ecs->attribute<Complex>("i_ref"), 0);
	intf.addExport(ecs->attribute<Complex>("v_comp"), 0);

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{ecs, r1});
	auto sim = RealTimeSimulation(simName, sys, timeStep, finalTime,
	Domain::DP, Solver::Type::MNA, Logger::Level::INFO);

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
