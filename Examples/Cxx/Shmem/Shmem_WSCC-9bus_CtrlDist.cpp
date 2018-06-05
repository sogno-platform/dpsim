/** CIM Test
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

#include <iostream>
#include <list>

#include "DPsim.h"
#include "cps/Interface.h"
#include "cps/CIM/Reader.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::Components::DP;

int main(int argc, char *argv[]) {

	CommandLineArgs args(argc, argv);

	Real timeStep = 0.001;
	Real finalTime = 20;
	String simName = "Shmem_WSCC-9bus_CtrlDist";

	if (args.scenario == 0) {

		// Specify CIM files
		String path("Examples/CIM/WSCC-09_Neplan_RX/");
		std::list<String> filenames = {
			path + "WSCC-09_Neplan_RX_DI.xml",
			path + "WSCC-09_Neplan_RX_EQ.xml",
			path + "WSCC-09_Neplan_RX_SV.xml",
			path + "WSCC-09_Neplan_RX_TP.xml"
		};

		CIM::Reader reader(simName, Logger::Level::INFO, Logger::Level::INFO);
		SystemTopology sys = reader.loadCIM(60, filenames);

		// Extend system with controllable load
		auto ecs = CurrentSource::make("i_intf", Node::List{sys.mNodes[3], GND}, Complex(0, 0), Logger::Level::DEBUG);
		sys.mComponents.push_back(ecs);

		RealTimeSimulation sim(simName + "_1", sys, timeStep, finalTime,
			Solver::Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG, true);

		// Create shmem interface and add it to simulation
		String in  = "/dpsim10";
		String out = "/dpsim01";
		Interface::Config conf;
		conf.samplelen = 64;
		conf.queuelen = 1024;
		conf.polling = false;
		Interface intf(out, in, &conf);
		sim.addInterface(&intf);

		// Register exportable node voltages		
		UInt o = 2;
		for (auto n : sys.mNodes) {
			auto v = n->findAttribute<Complex>("voltage");

			std::function<Real()> getMag = [v](){ return std::abs(v->get()); };
			std::function<Real()> getPhas = [v](){ return std::arg(v->get()); };

			intf.addExport(v, 1.0, o, o+1);
			intf.addExport(getMag, o+2);
			intf.addExport(getPhas, o+3);

			o += 4;
		}

		// Register interface current source and voltage drop
		intf.addImport(ecs->findAttribute<Complex>("current_ref"), 1.0, 0, 1);
		intf.addExport(ecs->findAttribute<Complex>("comp_voltage"), 1.0, 0, 1);

		sim.run(false, args.startTime);
	}

	if (args.scenario == 1) {
		// Nodes
		auto n1 = Node::make("n1", Complex(02.180675e+05, -1.583367e+04));

		// Add interface voltage source
		auto evs = VoltageSource::make("v_intf", Node::List{GND, n1}, Complex(0, 0), Logger::Level::DEBUG);

		// Extend system with controllable load
		auto load = PQLoadCS::make("load_cs", Node::List{n1}, 0, 0, 230000);

		auto sys = SystemTopology(60, Node::List{n1}, ComponentBase::List{evs, load});
		RealTimeSimulation sim(simName + "_2", sys, timeStep, finalTime);

		// Create shmem interface 1
		String in1  = "/dpsim01";
		String out1 = "/dpsim10";
		Interface::Config conf1;
		conf1.samplelen = 64;
		conf1.queuelen = 1024;
		conf1.polling = false;
		Interface intf1(out1, in1, &conf1);
		sim.addInterface(&intf1);

		// Create shmem interface 2
		String in2  = "/villas-dpsim";
		String out2 = "/dpsim-villas";
		Interface::Config conf2;
		conf2.samplelen = 64;
		conf2.queuelen = 1024;
		conf2.polling = false;
		Interface intf2(out2, in2, &conf2);
		sim.addInterface(&intf2);

		// Register voltage source reference and current flowing through source
		// multiply with -1 to consider passive sign convention
		intf1.addImport(evs->findAttribute<Complex>("voltage_ref"), 1.0, 0, 1);
		intf1.addExport(evs->findAttribute<Complex>("comp_current"), -1.0, 0, 1);

		// Register controllable load
		intf2.addImport(load->findAttribute<Real>("active_power"), 1.0, 0);
		intf2.addExport(load->findAttribute<Real>("active_power"), 1.0, 0);
		intf2.addExport(load->findAttribute<Complex>("comp_voltage"), 1.0, 1, 2);
		intf2.addExport(load->findAttribute<Complex>("comp_current"), 1.0, 3, 4);

		sim.run(false, args.startTime);
	}

	return 0;
}
