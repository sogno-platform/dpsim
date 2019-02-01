/** CIM Test
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <iostream>
#include <list>

#include <DPsim.h>
#include <dpsim/Interface.h>
#include <cps/CIM/Reader.h>

using namespace DPsim;
using namespace CPS;
using namespace CPS::DP::Ph1;
using namespace CPS::Signal;

int main(int argc, char *argv[]) {

	CommandLineArgs args(argc, argv, "Shmem_WSCC-9bus_CtrlDist", 0.001, 20, 60);

	if (args.scenario == 0) {

		// Specify CIM files
		String path("Examples/CIM/WSCC-09_RX/");
		std::list<String> filenames = {
			path + "WSCC-09_RX_DI.xml",
			path + "WSCC-09_RX_EQ.xml",
			path + "WSCC-09_RX_SV.xml",
			path + "WSCC-09_RX_TP.xml"
		};

		CIM::Reader reader(args.name, Logger::Level::INFO, Logger::Level::INFO);
		SystemTopology sys = reader.loadCIM(args.sysFreq, filenames);

		// Extend system with controllable load (Profile)
		auto load_profile = PQLoadCS::make("load_cs_profile", 0, 0, 230000, Logger::Level::INFO);
		load_profile->connect({ sys.node<DP::Node>("BUS7") });
		sys.mComponents.push_back(load_profile);

		// Extend system with controllable load
		auto ecs = CurrentSource::make("i_intf", Complex(0, 0), Logger::Level::DEBUG);
		ecs->connect({ sys.node<DP::Node>("BUS4"), DP::Node::GND });
		sys.mComponents.push_back(ecs);

		RealTimeSimulation sim(args.name + "_1", sys, args.timeStep, args.duration,
			Domain::DP, Solver::Type::MNA, Logger::Level::DEBUG, true);

		// Create shmem interface and add it to simulation
		String in  = "/dpsim10";
		String out = "/dpsim01";
		Interface::Config conf;
		conf.samplelen = 64;
		conf.queuelen = 1024;
		conf.polling = false;
		Interface intf1(out, in, &conf, false);
		sim.addInterface(&intf1);

		// Create shmem interface 2
		String in2  = "/villas-dpsim1";
		String out2 = "/dpsim1-villas";
		Interface::Config conf2;
		conf2.samplelen = 64;
		conf2.queuelen = 1024;
		conf2.polling = false;
		Interface intf2(out2, in2, &conf2, false);
		sim.addInterface(&intf2, false);

		// Controllers and filter
		std::vector<Real> coefficients_profile = std::vector<Real>(2000, 1./2000);

		auto filtP_profile = FIRFilter::make("filter_p_profile", coefficients_profile, 0, Logger::Level::INFO);
		filtP_profile->setPriority(1);
		load_profile->setAttributeRef("power_active", filtP_profile->attribute<Real>("output"));
		sys.mComponents.push_back(filtP_profile);

		// Register interface current source and voltage drop
		ecs->setAttributeRef("I_ref", intf1.importComplex(0));
		intf1.addExport(ecs->attribute<Complex>("v_comp"), 0);

		// TODO: gain by 20e8
		filtP_profile->setInput(intf2.importReal(0));

		// Register exportable node voltages
		for (auto n : sys.mNodes) {
			UInt i;
			if (sscanf(n->name().c_str(), "BUS%u", &i) != 1) {
				std::cerr << "Failed to determine bus no of bus: " << n->name() << std::endl;
				continue;
			}

			i--;

			auto v = n->attributeComplex("v");

			std::cout << "Signal " << (i*2)+0 << ": Mag " << n->name() << std::endl;
			std::cout << "Signal " << (i*2)+1 << ": Phas " << n->name() << std::endl;

			intf2.addExport(v->mag(),   (i*2)+0);
			intf2.addExport(v->phase(), (i*2)+1);
		}

		sim.run(args.startTime);
	}

	if (args.scenario == 1) {
		// Nodes
		auto n1 = DP::Node::make("n1", PhaseType::Single, std::vector<Complex>({Complex(02.180675e+05, -1.583367e+04)}));

		// Add interface voltage source
		auto evs = VoltageSource::make("v_intf", Logger::Level::DEBUG);
		evs->setParameters(Complex(0, 0));
		evs->connect({ DP::Node::GND, n1 });

		// Extend system with controllable load
		auto load = PQLoadCS::make("load_cs", 0, 0, 230000);
		load->connect({ n1 });

		// Controllers and filter
		std::vector<Real> coefficients = std::vector<Real>(100, 1./100);
		auto filtP = FIRFilter::make("filter_p", coefficients, 0, Logger::Level::INFO);
		filtP->setPriority(1);
		load->setAttributeRef("active_power", filtP->attribute<Real>("output"));

		auto sys = SystemTopology(args.sysFreq, SystemNodeList{n1}, SystemComponentList{evs, load, filtP});
		RealTimeSimulation sim(args.name + "_2", sys, args.timeStep, args.duration);

		// Create shmem interface 1
		String in1  = "/dpsim01";
		String out1 = "/dpsim10";
		Interface::Config conf1;
		conf1.samplelen = 64;
		conf1.queuelen = 1024;
		conf1.polling = false;
		Interface intf1(out1, in1, &conf1, false);
		sim.addInterface(&intf1);

		// Create shmem interface 2
		String in2  = "/villas-dpsim2";
		String out2 = "/dpsim2-villas";
		Interface::Config conf2;
		conf2.samplelen = 64;
		conf2.queuelen = 1024;
		conf2.polling = false;
		Interface intf2(out2, in2, &conf2, false);
		sim.addInterface(&intf2, false);

		// Register voltage source reference and current flowing through source
		// multiply with -1 to consider passive sign convention
		evs->setAttributeRef("V_ref", intf1.importComplex(0));
		// TODO: invalid sign
		intf1.addExport(evs->attribute<Complex>("i_comp"), 0);

		// Register controllable load
		filtP->setInput(intf2.importReal(0));
		intf2.addExport(load->attribute<Real>("power_active"), 0);
		intf2.addExport(load->attribute<Complex>("v_comp"), 1);
		intf2.addExport(load->attribute<Complex>("i_comp"), 2);

		sim.run(args.startTime);
	}

	return 0;
}
