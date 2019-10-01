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
	// Testing the interface with a simple circuit,
	// but the load is simulated in a different instance.
	// Values are exchanged using the ideal transformator model: an ideal
	// current source on the supply side and an ideal voltage source on the
	// supply side, whose values are received from the respective other circuit.
	// Here, the two instances directly communicate with each other without using
	// VILLASnode in between.

	if (argc < 2) {
		std::cerr << "not enough arguments (either 0 or 1 for the test number)" << std::endl;
		std::exit(1);
	}

	String in, out;

	if (String(argv[1]) == "0") {
		in  = "/dpsim10";
		out = "/dpsim01";
	}
	else if (String(argv[1]) == "1") {
		in  = "/dpsim01";
		out = "/dpsim10";
	} else {
		std::cerr << "invalid test number" << std::endl;
		std::exit(1);
	}

	Real timeStep = 0.001;
	Real finalTime = 0.1;

	if (String(argv[1]) == "0") {
		String simName = "ShmemDistributedDirect_1";
		Logger::setLogDir("logs/"+simName);

		// Nodes
		auto n1 = Node::make("n1", PhaseType::Single, std::vector<Complex>{ 10 });
		auto n2 = Node::make("n2", PhaseType::Single, std::vector<Complex>{ 5 });

		// Components
		auto evs = VoltageSource::make("v_intf", Logger::Level::debug);
		evs->setParameters(Complex(5, 0));
		auto vs1 = VoltageSource::make("vs_1", Logger::Level::debug);
		vs1->setParameters(Complex(10, 0));
		auto r12 = Resistor::make("r_12", Logger::Level::debug);
		r12->setParameters(1);

		// Connections
		evs->connect({ Node::GND, n2 });
		vs1->connect({ Node::GND, n1 });
		r12->connect({ n1, n2 });

		auto sys = SystemTopology(50,
			SystemNodeList{ n1, n2 },
			SystemComponentList{ evs, vs1, r12 });

		Simulation sim(simName);
		sim.setSystem(sys);
		sim.setTimeStep(timeStep);
		sim.setFinalTime(finalTime);

		// Logging
		auto logger = DataLogger::make(simName);
		logger->addAttribute("v1", n1->attribute("v"));
		logger->addAttribute("v2", n2->attribute("v"));
		logger->addAttribute("r12", r12->attribute("i_intf"));
		logger->addAttribute("ievs", evs->attribute("i_intf"));
		logger->addAttribute("vevs", evs->attribute("v_intf"));
		sim.addLogger(logger);

		// Map attributes to interface entries
		Interface intf(in, out);
		evs->setAttributeRef("V_ref", intf.importComplex(0));
		auto evsAttrMinus = evs->attributeMatrixComp("i_intf")->coeff(0,0);
		intf.exportComplex(evsAttrMinus, 0);
		sim.addInterface(&intf);

		MatrixComp initialEvsCurrent = MatrixComp::Zero(1,1);
		initialEvsCurrent(0,0) = Complex(5,0);
		evs->setIntfCurrent(initialEvsCurrent);

		sim.run();
	}
	else if (String(argv[1]) == "1") {
		String simName = "ShmemDistributedDirect_2";
		Logger::setLogDir("logs/"+simName);

		// Nodes
		auto n2 = Node::make("n2", PhaseType::Single, std::vector<Complex>{ 5 });

		// Components
		auto ecs = CurrentSource::make("i_intf", Logger::Level::debug);
		ecs->setParameters(Complex(5, 0));
		auto r02 = Resistor::make("r_02", Logger::Level::debug);
		r02->setParameters(1);

		// Connections
		ecs->connect({ Node::GND, n2 });
		r02->connect({ Node::GND, n2 });

		auto sys = SystemTopology(50,
			SystemNodeList{ n2 },
			SystemComponentList{ ecs, r02 });

		Simulation sim(simName);
		sim.setSystem(sys);
		sim.setTimeStep(timeStep);
		sim.setFinalTime(finalTime);

		// Logging
		auto logger = DataLogger::make(simName);
		logger->addAttribute("v2", n2->attribute("v"));
		logger->addAttribute("r02", r02->attribute("i_intf"));
		logger->addAttribute("vecs", ecs->attribute("v_intf"));
		logger->addAttribute("iecs", ecs->attribute("i_intf"));
		sim.addLogger(logger);

		// Map attributes to interface entries
		Interface intf(in, out);
		ecs->setAttributeRef("I_ref", intf.importComplex(0));
		//intf.exportComplex(ecs->attributeMatrixComp("v_intf")->coeff(0, 0), 0);
		intf.exportComplex(ecs->attributeMatrixComp("v_intf")->coeff(0, 0)->scale(Complex(-1.,0)), 0);
		sim.addInterface(&intf);

		sim.run();
	}
}
