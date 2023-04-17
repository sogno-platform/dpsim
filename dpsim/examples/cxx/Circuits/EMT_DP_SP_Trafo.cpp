/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/


#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

void simTrafoElementsSP1ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	Real frequency = 50;
	Real omega = 2. * PI * frequency;
	String simName = "SP_Trafo_Elements";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real trafoPower = 1e6;
	Real pSnub = P_SNUB_TRANSFORMER*trafoPower;
	Real qSnub = Q_SNUB_TRANSFORMER*trafoPower;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real snubberResistanceHVSide = std::pow(std::abs(voltageHVSide),2)/pSnub;
	Real snubberResistanceMVSideToHVSide = ratio*ratio*std::pow(std::abs(voltageMVSide),2)/pSnub;
	Real snubberCapacitanceMVSideToHVSide = 1./(omega*ratio*ratio*std::pow(std::abs(voltageMVSide),2)/qSnub);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");
	auto vn1 = SimNode<Complex>::make("vn1");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("v_1");
	auto trafoRes = SP::Ph1::Resistor::make("trafo_res");
	auto trafoSnubberResHVSide  = SP::Ph1::Resistor::make("trafo_snub_res_mv");
	auto trafoSnubberResMVSide  = SP::Ph1::Resistor::make("trafo_snub_res_hv");
	auto trafoSnubberCapMVSide  = SP::Ph1::Capacitor::make("trafo_snub_cap_mv");
	auto trafoInd = SP::Ph1::Inductor::make("trafo_ind");
	auto loadRes = SP::Ph1::Resistor::make("r_1");

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafoRes->connect({ n1, vn1 });
	trafoInd->connect({ vn1, n2 });
	trafoSnubberResHVSide->connect({ n1, SimNode<Complex>::GND });
	trafoSnubberResMVSide->connect({ n2, SimNode<Complex>::GND });
	trafoSnubberCapMVSide->connect({ n2, SimNode<Complex>::GND });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafoRes->setParameters(trafoResistance);
	trafoInd->setParameters(trafoInductance);
	trafoSnubberResHVSide->setParameters(snubberResistanceHVSide);
	trafoSnubberResMVSide->setParameters(snubberResistanceMVSideToHVSide);
	trafoSnubberCapMVSide->setParameters(snubberCapacitanceMVSideToHVSide);
	loadRes->setParameters(loadResistanceHVSide);

	// Define system topology
	SystemTopology sys(frequency, SystemNodeList{n1, n2, vn1 }, SystemComponentList{vs, trafoRes, trafoInd, trafoSnubberResHVSide, trafoSnubberResMVSide, trafoSnubberCapMVSide, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafoInd->attribute("i_intf"));

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
	Real trafoPower = 1e6;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real loadResistanceMVSide = loadResistanceHVSide/(ratio*ratio);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");

	// Components
	auto vs = SP::Ph1::VoltageSource::make("v_1", Logger::Level::debug);
	auto trafo = SP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug);
	auto loadRes = SP::Ph1::Resistor::make("r_1", Logger::Level::debug);

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafo->connect({ n1, n2 });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafo->setParameters(voltageHVSide, voltageMVSide, trafoPower, ratio, 0, trafoResistance, trafoInductance);
	loadRes->setParameters(loadResistanceMVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2 }, SystemComponentList{vs, trafo, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafo->attribute("i_intf"));

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
	Real frequency = 50;
	Real omega = 2. * PI * frequency;
	String simName = "DP_Trafo_Elements";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real trafoPower = 1e6;
	Real pSnub = P_SNUB_TRANSFORMER*trafoPower;
	Real qSnub = Q_SNUB_TRANSFORMER*trafoPower;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real snubberResistanceHVSide = std::pow(std::abs(voltageHVSide),2)/pSnub;
	Real snubberResistanceMVSideToHVSide = ratio*ratio*std::pow(std::abs(voltageMVSide),2)/pSnub;
	Real snubberCapacitanceMVSideToHVSide = 1./(omega*ratio*ratio*std::pow(std::abs(voltageMVSide),2)/qSnub);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1");
	auto n2 = SimNode<Complex>::make("n2");
	auto vn1 = SimNode<Complex>::make("vn1");

	// Components
	auto vs = DP::Ph1::VoltageSource::make("v_1");
	auto trafoRes = DP::Ph1::Resistor::make("trafo_res");
	auto trafoSnubberResHVSide  = DP::Ph1::Resistor::make("trafo_snub_res_mv");
	auto trafoSnubberResMVSide  = DP::Ph1::Resistor::make("trafo_snub_res_hv");
	auto trafoSnubberCapMVSide  = DP::Ph1::Capacitor::make("trafo_snub_cap_mv");
	auto trafoInd = DP::Ph1::Inductor::make("trafo_ind");
	auto loadRes = DP::Ph1::Resistor::make("r_1");

	// Topology
	vs->connect({ SimNode<Complex>::GND, n1 });
	trafoRes->connect({ n1, vn1 });
	trafoInd->connect({ vn1, n2 });
	trafoSnubberResHVSide->connect({ n1, SimNode<Complex>::GND });
	trafoSnubberResMVSide->connect({ n2, SimNode<Complex>::GND });
	trafoSnubberCapMVSide->connect({ n2, SimNode<Complex>::GND });
	loadRes->connect({ n2, SimNode<Complex>::GND });

	// Parameters
	vs->setParameters(CPS::Math::polar(voltageHVSide, 0));
	trafoRes->setParameters(trafoResistance);
	trafoInd->setParameters(trafoInductance);
	trafoSnubberResHVSide->setParameters(snubberResistanceHVSide);
	trafoSnubberResMVSide->setParameters(snubberResistanceMVSideToHVSide);
	trafoSnubberCapMVSide->setParameters(snubberCapacitanceMVSideToHVSide);
	loadRes->setParameters(loadResistanceHVSide);

	// Define system topology
	SystemTopology sys(frequency, SystemNodeList{n1, n2, vn1 }, SystemComponentList{vs, trafoRes, trafoInd, trafoSnubberResHVSide, trafoSnubberResMVSide, trafoSnubberCapMVSide, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafoInd->attribute("i_intf"));

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
	Real trafoPower = 1e6;
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
	trafo->setParameters(voltageHVSide, voltageMVSide, trafoPower, ratio, 0, trafoResistance, trafoInductance);
	loadRes->setParameters(loadResistanceMVSide);

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2 }, SystemComponentList{vs, trafo, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafo->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	sim.run();
}

void simTrafoElementsEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	Real frequency = 50;
	Real omega = 2. * PI * frequency;
	String simName = "EMT_Trafo_Elements";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real trafoPower = 1e6;
	Real pSnub = P_SNUB_TRANSFORMER*trafoPower;
	Real qSnub = Q_SNUB_TRANSFORMER*trafoPower;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real snubberResistanceHVSide = std::pow(std::abs(voltageHVSide),2)/pSnub;
	Real snubberResistanceMVSideToHVSide = ratio*ratio*std::pow(std::abs(voltageMVSide),2)/pSnub;
	Real snubberCapacitanceMVSideToHVSide = 1./(omega*ratio*ratio*std::pow(std::abs(voltageMVSide),2)/qSnub);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);
	auto vn1 = SimNode<Real>::make("vn1", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("v_1");
	auto trafoRes = EMT::Ph3::Resistor::make("trafo_res");
	auto trafoSnubberResHVSide  = EMT::Ph3::Resistor::make("trafo_snub_res_mv");
	auto trafoSnubberResMVSide  = EMT::Ph3::Resistor::make("trafo_snub_res_hv");
	auto trafoSnubberCapMVSide  = EMT::Ph3::Capacitor::make("trafo_snub_cap_mv");
	auto trafoInd = EMT::Ph3::Inductor::make("trafo_ind");
	auto loadRes = EMT::Ph3::Resistor::make("r_1");

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	trafoRes->connect({ n1, vn1 });
	trafoInd->connect({ vn1, n2 });
	trafoSnubberResHVSide->connect({ n1, SimNode<Real>::GND });
	trafoSnubberResMVSide->connect({ n2, SimNode<Real>::GND });
	trafoSnubberCapMVSide->connect({ n2, SimNode<Real>::GND });
	loadRes->connect({ n2, SimNode<Real>::GND });

	// Parameters
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);
	trafoRes->setParameters(CPS::Math::singlePhaseParameterToThreePhase(trafoResistance));
	trafoInd->setParameters(CPS::Math::singlePhaseParameterToThreePhase(trafoInductance));
	trafoSnubberResHVSide->setParameters(CPS::Math::singlePhaseParameterToThreePhase(snubberResistanceHVSide));
	trafoSnubberResMVSide->setParameters(CPS::Math::singlePhaseParameterToThreePhase(snubberResistanceMVSideToHVSide));
	trafoSnubberCapMVSide->setParameters(CPS::Math::singlePhaseParameterToThreePhase(snubberCapacitanceMVSideToHVSide));
	loadRes->setParameters(CPS::Math::singlePhaseParameterToThreePhase(loadResistanceHVSide));

	// Define system topology
	SystemTopology sys(frequency, SystemNodeList{n1, n2, vn1 }, SystemComponentList{vs, trafoRes, trafoInd, trafoSnubberResHVSide, trafoSnubberResMVSide, trafoSnubberCapMVSide, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafoInd->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

void simTrafoEMT3ph() {
	Real timeStep = 0.00005;
	Real finalTime = 1;
	String simName = "EMT_Trafo_Component";
	Logger::setLogDir("logs/"+simName);

	Real voltageHVSide = 100000;
	Real voltageMVSide = 10000;
	Real trafoResistance = 1;
	Real trafoInductance = 0.1;
	Real trafoPower = 1e6;
	Real loadResistanceHVSide = 10000;
	Real ratio = voltageHVSide/voltageMVSide;
	Real loadResistanceMVSide = loadResistanceHVSide/(ratio*ratio);

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

	// Components
	auto vs = EMT::Ph3::VoltageSource::make("v_1", Logger::Level::debug);
	auto trafo = EMT::Ph3::Transformer::make("trafo", "trafo", Logger::Level::debug, true);
	auto loadRes = EMT::Ph3::Resistor::make("r_1", Logger::Level::debug);

	// Topology
	vs->connect({ SimNode<Real>::GND, n1 });
	trafo->connect({ n1, n2 });
	loadRes->connect({ n2, SimNode<Real>::GND });

	// Parameters
	vs->setParameters(CPS::Math::singlePhaseVariableToThreePhase(CPS::Math::polar(100000, 0)), 50);
	trafo->setParameters(voltageHVSide, voltageMVSide, trafoPower, ratio, 0, CPS::Math::singlePhaseParameterToThreePhase(trafoResistance), CPS::Math::singlePhaseParameterToThreePhase(trafoInductance));
	loadRes->setParameters(CPS::Math::singlePhaseParameterToThreePhase(loadResistanceMVSide));

	// Define system topology
	SystemTopology sys(50, SystemNodeList{n1, n2 }, SystemComponentList{vs, trafo, loadRes});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("itrafo", trafo->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}


int main(int argc, char* argv[]) {
	simTrafoElementsSP1ph();
	simTrafoSP1ph();

	simTrafoElementsDP1ph();
	simTrafoDP1ph();

	simTrafoElementsEMT3ph();
	simTrafoEMT3ph();

	return 0;
}
