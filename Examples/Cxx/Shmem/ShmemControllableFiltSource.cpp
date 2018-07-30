/** Example of shared memory interface
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
using namespace CPS;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

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

	// Controllers and filter
	std::vector<Real> coefficients = { -0.0024229,-0.0020832,0.0067703,0.016732,
	0.011117,-0.0062311,-0.0084016,0.0092568, 0.012983,-0.010121,-0.018274,0.011432,
	0.026176,-0.012489,-0.037997,0.013389,0.058155,-0.014048,-0.10272,0.014462,0.31717,
	0.48539, 0.31717,0.014462,-0.10272,-0.014048,0.058155,0.013389,-0.037997,-0.012489,
	0.026176,0.011432,-0.018274,-0.010121, 0.012983,0.0092568,-0.0084016,-0.0062311,
	0.011117,0.016732,0.0067703,-0.0020832,-0.0024229 };

	auto filtP = FIRFilter::make("filter_p", coefficients, Logger::Level::DEBUG);
	auto filtQ = FIRFilter::make("filter_q", coefficients, Logger::Level::DEBUG);

	filtP->setPriority(1);
	filtQ->setPriority(1);
	filtP->initialize(10.);
	filtQ->initialize(0);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto ecs = CurrentSource::make("v_intf", Node::List{GND, n1}, Complex(10, 0));
	auto r1 = Resistor::make("r_1", Node::List{GND, n1}, 1);
	auto load = PQLoadCS::make("load_cs", Node::List{n1}, 10., 0., 10.);

	filtP->setConnection(load->findAttribute<Real>("active_power"));
	filtQ->setConnection(load->findAttribute<Real>("reactive_power"));

	filtP->findAttribute<Real>("input")->set(8.);
	filtQ->findAttribute<Real>("input")->set(0.);

	intf.addImport(filtP->findAttribute<Real>("input"), 1.0, 0);
	intf.addImport(filtQ->findAttribute<Real>("input"), 1.0, 1);

	auto sys = SystemTopology(50, Node::List{n1}, ComponentBase::List{ecs, r1, load, filtP, filtQ});
	auto sim = RealTimeSimulation(simName, sys, timeStep, finalTime,
	Domain::DP, Solver::Type::MNA, Logger::Level::INFO);

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
