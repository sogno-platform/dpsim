/** Example of shared memory interface
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

int main(int argc, char* argv[])
{
	// Same circuit as above, but now with realtime support.
	SystemComponentList comps;

	CPS::Interface::Config conf;
	conf.samplelen = 4;
	conf.queuelen = 1024;
	conf.polling = false;

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");

	// Components
	auto evs = VoltageSource::make("v_s");
	auto rs =  Resistor::make("r_s");
	auto rl =  Resistor::make("r_line");
	auto ll =  Inductor::make("l_line");
	auto rL =  Resistor::make("r_load");

	// Topology
	evs->connect({ Node::GND, n1 });
	rs->connect({ n1, n2 });
	rl->connect({ n2, n3 });
	ll->connect({ n3, n4 });
	rL->connect({ n4, Node::GND });

	// Parameters
	evs->setParameters(Complex(0, 0));
	rs->setParameters(1);
	rl->setParameters(1);
	ll->setParameters(1);
	rL->setParameters(1000);

	auto intf = CPS::Interface("/villas1-in", "/villas1-out", &conf);

	intf.addImport(evs->attribute<Complex>("v_ref"), 0);
	intf.addExport(evs->attribute<Complex>("i_comp"), 0);

	Real timeStep = 0.001;
	auto sys = SystemTopology(50, SystemNodeList{Node::GND, n1, n2, n3, n4}, SystemComponentList{evs, rs, rl, ll, rL});
	auto sim = RealTimeSimulation("ShmemRealTime", sys, timeStep, 5.0);

	sim.addInterface(&intf, false, true);
	sim.run();

	return 0;
}
