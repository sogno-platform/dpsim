/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;

// Define machine parameters in per unit
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real nomFreq = 60;
Real nomFieldCurr = 1300;
Int poleNum = 2;
Real H = 3.7;
Real Rs = 0.003;
Real Ll = 0.15;
Real Lmd = 1.6599;
Real Lmq = 1.61;
Real Rfd = 0.0006;
Real Llfd = 0.1648;
Real Rkd = 0.0284;
Real Llkd = 0.1713;
Real Rkq1 = 0.0062;
Real Llkq1 = 0.7252;
Real Rkq2 = 0.0237;
Real Llkq2 = 0.125;
// Initialization parameters
Real initActivePower = 300e6;
Real initReactivePower = 0;
Real initMechPower = 300e6;
Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
Real initVoltAngle = -PI / 2;

// Define grid parameters
// resistance for 300 MW output
Real Rload = 1.92;

// Initial node voltage
auto initVoltN1 = std::vector<Complex>({
	Complex(initTerminalVolt * cos(initVoltAngle),
		initTerminalVolt * sin(initVoltAngle)),
	Complex(initTerminalVolt * cos(initVoltAngle - 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle - 2 * PI / 3)),
	Complex(initTerminalVolt * cos(initVoltAngle + 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle + 2 * PI / 3)) });

void DP_SynGenDq7odTrapez_SteadyState(Real timeStep, Real finalTime) {
	String simName = "DP_SynGenDq7odTrapez_SteadyState";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::DP::SimNode::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::DP::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initMechPower);

	auto res = CPS::DP::Ph3::SeriesResistor::make("R_load", Logger::Level::info);
	res->setParameters(Rload);

	// Connections
	gen->connect({n1});
	res->connect({CPS::DP::SimNode::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i_load", res->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	sim.run();
}

void EMT_SynGenDq7odTrapez_SteadyState(Real timeStep, Real finalTime) {
	String simName = "EMT_SynGenDq7odTrapez_SteadyState";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::SimNode::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, initMechPower);

	auto res = CPS::EMT::Ph3::SeriesResistor::make("R_load", Logger::Level::info);
	res->setParameters(Rload);

	// Connections
	gen->connect({n1});
	res->connect({CPS::EMT::SimNode::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i_load", res->attribute("i_intf"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {

	Real finalTime = 0.3;
	Real timeStep = 0.00005;
	DP_SynGenDq7odTrapez_SteadyState(timeStep, finalTime);
	EMT_SynGenDq7odTrapez_SteadyState(timeStep, finalTime);
}
