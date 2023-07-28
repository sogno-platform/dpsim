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
Real BreakerOpen = 1e6;
Real BreakerClosed = 0.001;

// Initial node voltage
auto initVoltN1 = std::vector<Complex>({
	Complex(initTerminalVolt * cos(initVoltAngle),
		initTerminalVolt * sin(initVoltAngle)),
	Complex(initTerminalVolt * cos(initVoltAngle - 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle - 2 * PI / 3)),
	Complex(initTerminalVolt * cos(initVoltAngle + 2 * PI / 3),
		initTerminalVolt * sin(initVoltAngle + 2 * PI / 3)) });

void DP_SynGenDq7odODE_LoadStep(Real timeStep, Real finalTime, Real breakerClosed, String extension = "") {
	String simName = "DP_SynGenDq7odODE" + extension;
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::DP::SimNode::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::DP::Ph3::SynchronGeneratorDQODE::make("SynGen");
	gen->setParametersFundamentalPerUnit(
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt,
		initVoltAngle, initMechPower);

	auto res = CPS::DP::Ph3::SeriesResistor::make("R_load");
	res->setParameters(Rload);

	auto fault = CPS::DP::Ph3::SeriesSwitch::make("Br_fault");
	fault->setParameters(BreakerOpen, breakerClosed);
	fault->open();

	// Connections
	gen->connect({n1});
	res->connect({CPS::DP::SimNode::GND, n1});
	fault->connect({CPS::DP::SimNode::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res, fault});

	// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	// Events
	if (breakerClosed > 0.0001) {
		auto sw1 = SwitchEvent::make(0.1, fault, true);
		sim.addEvent(sw1);
		auto sw2 = SwitchEvent::make(0.2, fault, false);
		sim.addEvent(sw2);
	}

	sim.run();
}

void EMT_SynGenDq7odODE_LoadStep(Real timeStep, Real finalTime, Real breakerClosed, String extension = "") {
	String simName = "EMT_SynGenDq7odODE" + extension;
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::SimNode::make("n1", PhaseType::ABC, initVoltN1);

	// Components
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQODE::make("SynGen");
	gen->setParametersFundamentalPerUnit(
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt,
		initVoltAngle, initMechPower);

	auto res = CPS::EMT::Ph3::SeriesResistor::make("R_load");
	res->setParameters(Rload);

	auto fault = CPS::EMT::Ph3::SeriesSwitch::make("Br_fault");
	fault->setParameters(BreakerOpen, breakerClosed);
	fault->open();

	// Connections
	gen->connect({n1});
	res->connect({CPS::EMT::SimNode::GND, n1});
	fault->connect({CPS::EMT::SimNode::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, res, fault});

		// Logging
	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.doInitFromNodesAndTerminals(false);
	sim.addLogger(logger);

	// Events
	if (breakerClosed > 0.0001) {
		auto sw1 = SwitchEvent::make(0.1, fault, true);
		sim.addEvent(sw1);
		auto sw2 = SwitchEvent::make(0.2, fault, false);
		sim.addEvent(sw2);
	}

	sim.run();
}

int main(int argc, char* argv[]) {
	Real finalTime = 0.3;
	Real timeStep = 0.00005;
	Real breakerOpenR = BreakerOpen;
	UInt maxTimeStepIdx = 20;
	UInt maxLoadStepIdx = 10;

	for (UInt loadStepIdx = 0; loadStepIdx <= maxLoadStepIdx; ++loadStepIdx) {
		if (loadStepIdx == 0) breakerOpenR = 0;
		else breakerOpenR = Rload / loadStepIdx;

		for (UInt stepIdx = 1; stepIdx <= maxTimeStepIdx; ++stepIdx) {
			timeStep = stepIdx * 0.00005;

			DP_SynGenDq7odODE_LoadStep(timeStep, finalTime, breakerOpenR,
				"_T" + std::to_string(stepIdx) + "_L" + std::to_string(loadStepIdx));
			EMT_SynGenDq7odODE_LoadStep(timeStep, finalTime, breakerOpenR,
				"_T" + std::to_string(stepIdx) + "_L" + std::to_string(loadStepIdx));
		}
	}
}
