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
using namespace CPS::Components;
using namespace CPS::Components::DP;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {

	CommandLineArgs args(argc, argv, 0.001, 20, 60);

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
		SystemTopology sys = reader.loadCIM(args.sysFreq, filenames);

		// Extend system with controllable load (Profile)
		auto load_profile = PQLoadCS::make("load_cs_profile", ComplexNode::List{sys.getDPNodeAt(6)}, 0, 0, 230000, Logger::Level::INFO);
		sys.mComponents.push_back(load_profile);

		// Extend system with controllable load
		auto ecs = CurrentSource::make("i_intf", ComplexNode::List{sys.getDPNodeAt(3), ComplexNode::GND}, Complex(0, 0), Logger::Level::DEBUG);
		sys.mComponents.push_back(ecs);

		RealTimeSimulation sim(simName + "_1", sys, args.timeStep, args.duration,
			Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG, true);

		// Create shmem interface and add it to simulation
		String in  = "/dpsim10";
		String out = "/dpsim01";
		Interface::Config conf;
		conf.samplelen = 64;
		conf.queuelen = 1024;
		conf.polling = false;
		Interface intf1(out, in, &conf);
		sim.addInterface(&intf1, false, true);

		// Create shmem interface 2
		String in2  = "/villas-dpsim1";
		String out2 = "/dpsim1-villas";
		Interface::Config conf2;
		conf2.samplelen = 64;
		conf2.queuelen = 1024;
		conf2.polling = false;
		Interface intf2(out2, in2, &conf2);
		sim.addInterface(&intf2, false, false);

		// Controllers and filter
		std::vector<Real> coefficients_profile = std::vector(2000, 1./2000);

		auto filtP_profile = FIRFilter::make("filter_p_profile", coefficients_profile, Logger::Level::INFO);
		filtP_profile->setPriority(1);
		filtP_profile->initialize(0.);
		filtP_profile->setConnection(load_profile->findAttribute<Real>("active_power"));
		filtP_profile->findAttribute<Real>("input")->set(0.);
		sys.mComponents.push_back(filtP_profile);

		// Register interface current source and voltage drop
		intf1.addImport(ecs->findAttribute<Complex>("current_ref"), 1.0, 0, 1);
		intf1.addExport(ecs->findAttribute<Complex>("comp_voltage"), 1.0, 0, 1);

		intf2.addImport(filtP_profile->findAttribute<Real>("input"), 20e8, 0);

		// Register exportable node voltages
		for (auto n : sys.mNodes) {
			UInt i;
			if (sscanf(n->getName().c_str(), "BUS%u", &i) != 1) {
				std::cerr << "Failed to determine bus no of bus: " << n->getName() << std::endl;
				continue;
			}

			i--;

			auto v = n->findAttribute<Complex>("voltage");

			std::cout << "Signal << " << (i*2)+0 << ": Mag " << n->getName() << std::endl;
			std::cout << "Signal << " << (i*2)+1 << ": Phas " << n->getName() << std::endl;

			std::function<Real()> getMag = [v](){ return std::abs(v->get()); };
			std::function<Real()> getPhas = [v](){ return std::arg(v->get()); };

			intf2.addExport(getMag,  (i*2)+0);
			intf2.addExport(getPhas, (i*2)+1);
		}

		sim.run(args.startTime);
	}

	if (args.scenario == 1) {
		// Nodes
		auto n1 = ComplexNode::make("n1", Complex(02.180675e+05, -1.583367e+04));

		// Add interface voltage source
		auto evs = VoltageSource::make("v_intf", ComplexNode::List{ComplexNode::GND, n1}, Complex(0, 0), Logger::Level::DEBUG);

		// Extend system with controllable load
		auto load = PQLoadCS::make("load_cs", ComplexNode::List{n1}, 0, 0, 230000);

		// Controllers and filter
		std::vector<Real> coefficients = std::vector(100, 1./100);
		auto filtP = FIRFilter::make("filter_p", coefficients, Logger::Level::INFO);
		filtP->setPriority(1);
		filtP->initialize(0.);
		filtP->setConnection(load->findAttribute<Real>("active_power"));
		filtP->findAttribute<Real>("input")->set(0.);

		auto sys = SystemTopology(args.sysFreq, NodeBase::List{n1}, ComponentBase::List{evs, load, filtP});
		RealTimeSimulation sim(simName + "_2", sys, args.timeStep, args.duration);

		// Create shmem interface 1
		String in1  = "/dpsim01";
		String out1 = "/dpsim10";
		Interface::Config conf1;
		conf1.samplelen = 64;
		conf1.queuelen = 1024;
		conf1.polling = false;
		Interface intf1(out1, in1, &conf1);
		sim.addInterface(&intf1, false, true);

		// Create shmem interface 2
		String in2  = "/villas-dpsim2";
		String out2 = "/dpsim2-villas";
		Interface::Config conf2;
		conf2.samplelen = 64;
		conf2.queuelen = 1024;
		conf2.polling = false;
		Interface intf2(out2, in2, &conf2);
		sim.addInterface(&intf2, false, false);

		// Register voltage source reference and current flowing through source
		// multiply with -1 to consider passive sign convention
		intf1.addImport(evs->findAttribute<Complex>("voltage_ref"), 1.0, 0, 1);
		intf1.addExport(evs->findAttribute<Complex>("comp_current"), -1.0, 0, 1);

		// Register controllable load
		intf2.addImport(filtP->findAttribute<Real>("input"), 1.0, 0);
		intf2.addExport(load->findAttribute<Real>("active_power"), 1.0, 0);
		intf2.addExport(load->findAttribute<Complex>("comp_voltage"), 1.0, 1, 2);
		intf2.addExport(load->findAttribute<Complex>("comp_current"), 1.0, 3, 4);

		sim.run(args.startTime);
	}

	return 0;
}
