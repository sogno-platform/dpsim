/**
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

int main(int argc, char* argv[]) {
	// Define simulation scenario
	Real timeStep = 0.000001;
	Real finalTime = 0.01;
	String simName = "DP_Inverter_Grid_Test";
	Logger::setLogDir("logs/"+simName);

	// Set system frequencies
	Matrix frequencies(5,1);
	frequencies << 50, 19850, 19950, 20050, 20150;
	//Matrix frequencies(1,1);
	//frequencies << 50;

	// Nodes
	auto n1 = Node::make("n1");
	auto n2 = Node::make("n2");
	auto n3 = Node::make("n3");
	auto n4 = Node::make("n4");
	auto n5 = Node::make("n5");

	// Components
	auto inv = Inverter::make("inv", Logger::Level::DEBUG);
	inv->setParameters(2, 3, 360, 0.87);
	//auto inv = VoltageSource::make("inv", Logger::Level::INFO);
	//inv->setParameters(Complex(0, -200));
	auto r1 = Resistor::make("r1", Logger::Level::INFO);
	r1->setParameters(0.1);
	auto l1 = Inductor::make("l1", Logger::Level::INFO);
	l1->setParameters(600e-6);
	auto r2 = Resistor::make("r2", Logger::Level::INFO);
	Real r2g = 0.1+0.001;
	r2->setParameters(r2g);
	auto l2 = Inductor::make("l2", Logger::Level::INFO);
	Real l2g = 150e-6+0.001/(2.*PI*50.);
	l2->setParameters(l2g);
	auto c1 = Capacitor::make("c1", Logger::Level::INFO);
	c1->setParameters(10e-6);
	auto rc = Capacitor::make("rc", Logger::Level::INFO);
	rc->setParameters(1e-6);
	auto grid = VoltageSource::make("grid", Logger::Level::INFO);
	grid->setParameters(Complex(0, -311.1270));

	// Topology
	//inv->connect({ Node::GND, n1 });
	inv->connect({ n1 });
	r1->connect({ n1, n2 });
	l1->connect({ n2, n3 });
	c1->connect({ Node::GND, n3 });
	rc->connect({ Node::GND, n3 });
	r2->connect({ n3, n4 });
	l2->connect({ n4, n5 });
	grid->connect({ Node::GND, n5 });

	// Define system topology
	auto sys = SystemTopology(50, frequencies,
		SystemNodeList{ n1, n2, n3, n4, n5 },
		SystemComponentList{ inv, r1, l1, r2, l2, c1, rc, grid });

	Simulation sim(simName, sys, timeStep, finalTime, Domain::DP, Solver::Type::MNA, Logger::Level::INFO);

	// Logging
	auto logger = DataLogger::make(simName);
	//logger->addAttribute("v1", n1->attributeMatrix<Complex>("v")->coeff(0,0));
	//logger->addAttribute("v1_1", n1->attributeMatrix<Complex>("v")->coeff(0,1));
	//logger->addAttribute("v1_2", n1->attributeMatrix<Complex>("v")->coeff(0,2));
	//logger->addAttribute("v1_3", n1->attributeMatrix<Complex>("v")->coeff(0,3));
	//logger->addAttribute("v1_4", n1->attributeMatrix<Complex>("v")->coeff(0,4));
	//logger->addAttribute("v2", n2->attributeMatrix<Complex>("v")->coeff(0,0));
	//logger->addAttribute("v3", n3->attributeMatrix<Complex>("v")->coeff(0,0));
	//logger->addAttribute("v3_1", n3->attributeMatrix<Complex>("v")->coeff(0,1));
	//logger->addAttribute("v3_2", n3->attributeMatrix<Complex>("v")->coeff(0,2));
	//logger->addAttribute("v3_3", n3->attributeMatrix<Complex>("v")->coeff(0,3));
	//logger->addAttribute("v3_4", n3->attributeMatrix<Complex>("v")->coeff(0,4));
	//logger->addAttribute("v4", n4->attributeMatrix<Complex>("v")->coeff(0,0));
	//logger->addAttribute("v5", n5->attributeMatrix<Complex>("v")->coeff(0,0));
//
	//auto test = r1->attributeMatrix<Complex>("i_intf")->coeff(0,0);
	//std::cout << test->get() << std::endl;
//
	//logger->addAttribute("i12", r1->attributeMatrix<Complex>("i_intf")->coeff(0,0));
	logger->addAttribute("i34", r2->attributeMatrix<Complex>("i_intf")->coeff(0,0));

	sim.addLogger(logger);

	sim.run();

	return 0;
}
