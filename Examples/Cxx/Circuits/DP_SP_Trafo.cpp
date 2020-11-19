/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/


#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void simTrafoElementsSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_Trafo_Elements";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real snubberResistanceHVSide = ratio*ratio*voltageMVSide*1e6;

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");
	auto vn1 = SimNode<Complex>::make("vn1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("v_1");
	auto trafoRes = SP::Ph1::Resistor::make("trafo_res");
	auto trafoSnubberRes  = SP::Ph1::Resistor::make("trafo_snub_res");
	auto trafoInd = SP::Ph1::Inductor::make("trafo_ind");
	auto loadRes = SP::Ph1::Resistor::make("r_1");	

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafoRes->connect({ n1, vn1 });
	trafoInd->connect({ vn1, n2 });
	trafoSnubberRes->connect({ n2, SimNode<Complex>::GND });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafoRes->setParameters(trafoResistance);
	trafoInd->setParameters(trafoInductance);
	trafoSnubberRes->setParameters(snubberResistanceHVSide);
	loadRes->setParameters(loadResistanceHVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2, vn1 }, SystemComponentList{vs, trafoRes, trafoInd, trafoSnubberRes, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("itrafo", trafoInd->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);

	sim.run();
}

void simTrafoSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "SP_Trafo_Component";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real loadResistanceMVSide = loadResistanceHVSide/(ratio*ratio);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("v_1", Logger::Level::debug);
	auto trafo = SP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug, true);
	auto loadRes = SP::Ph1::Resistor::make("r_1", Logger::Level::debug);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafo->connect({ n1, n2 });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafo->setParameters(voltageHVSide, voltageMVSide, 50e6, ratio, 0, trafoResistance, trafoInductance, 2.0*PI*50.0);
	loadRes->setParameters(loadResistanceMVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2 }, SystemComponentList{vs, trafo, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("itrafo", trafo->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);

	sim.run();
}

void simTrafoElementsDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Trafo_Elements";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real snubberResistanceHVSide = ratio*ratio*voltageMVSide*1e6;

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");
	auto vn1 = SimNode<Complex>::make("vn1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("v_1");
	auto trafoRes = DP::Ph1::Resistor::make("trafo_res");
	auto trafoSnubberRes  = DP::Ph1::Resistor::make("trafo_snub_res");
	auto trafoInd = DP::Ph1::Inductor::make("trafo_ind");
	auto loadRes = DP::Ph1::Resistor::make("r_1");	

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafoRes->connect({ n1, vn1 });
	trafoInd->connect({ vn1, n2 });
	trafoSnubberRes->connect({ n2, SimNode<Complex>::GND });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafoRes->setParameters(trafoResistance);
	trafoInd->setParameters(trafoInductance);
	trafoSnubberRes->setParameters(snubberResistanceHVSide);
	loadRes->setParameters(loadResistanceHVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2, vn1 }, SystemComponentList{vs, trafoRes, trafoInd, trafoSnubberRes, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("itrafo", trafoInd->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	sim.run();
}

void simTrafoDP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "DP_Trafo_Component";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real loadResistanceMVSide = loadResistanceHVSide/(ratio*ratio);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("v_1", Logger::Level::debug);
	auto trafo = DP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug, true);
	auto loadRes = DP::Ph1::Resistor::make("r_1", Logger::Level::debug);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafo->connect({ n1, n2 });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafo->setParameters(voltageHVSide, voltageMVSide, ratio, 0, trafoResistance, trafoInductance);
	loadRes->setParameters(loadResistanceMVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2 }, SystemComponentList{vs, trafo, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("itrafo", trafo->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	sim.run();
}


int main(int argc, char* argv[]) {
	simTrafoElementsSP1ph();
	simTrafoSP1ph();

	simTrafoElementsDP1ph();
	simTrafoDP1ph();

	return 0;
}
