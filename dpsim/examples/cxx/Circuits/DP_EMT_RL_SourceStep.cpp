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

static void DP_RL_SourceStep(Real timeStep, Real finalTime,
	Real voltageStep, Real freqStep, Real stepTime, String extension = "") {
	String simName = "DP_RL_SourceStep" + extension;
	Logger::setLogDir("logs/"+simName);

	auto n1 = DP::SimNode::make("n1");
	auto n2 = DP::SimNode::make("n2");
	auto n3 = DP::SimNode::make("n3");

	auto vs = std::make_shared<DP::Ph1::VoltageSourceRamp>("v_s");
	vs->setParameters(1000, voltageStep, 0, 2*PI*freqStep, stepTime, 0);
	auto r = DP::Ph1::Resistor::make("r_line");
	r->setParameters(1);
	auto l = DP::Ph1::Inductor::make("l_line");
	l->setParameters(0.2);
	auto sw = DP::Ph1::Resistor::make("r_load");
	sw->setParameters(100);

	vs->connect({DP::SimNode::GND, n1});
	r->connect({n1, n2});
	l->connect({n2, n3});
	sw->connect({DP::SimNode::GND, n3});

	auto sys = SystemTopology(50,
		SystemNodeList{ n1, n2, n3 },
		SystemComponentList{ vs, r, l, sw });

	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_r", r->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

static void EMT_RL_SourceStep(Real timeStep, Real finalTime,
	Real voltageStep, Real freqStep, Real stepTime, String extension = "") {
	String simName = "EMT_RL_SourceStep" + extension;
	Logger::setLogDir("logs/"+simName);

	auto n1 = EMT::SimNode::make("n1");
	auto n2 = EMT::SimNode::make("n2");
	auto n3 = EMT::SimNode::make("n3");

	auto vs = std::make_shared<EMT::Ph1::VoltageSourceRamp>("v_s");
	vs->setParameters(1000, voltageStep, 2*PI*50, 2*PI*freqStep, stepTime, 0);
	auto r = EMT::Ph1::Resistor::make("r_line");
	r->setParameters(1);
	auto l = EMT::Ph1::Inductor::make("l_line");
	l->setParameters(0.2);
	auto sw = EMT::Ph1::Resistor::make("r_load");
	sw->setParameters(100);

	vs->connect({EMT::SimNode::GND, n1});
	r->connect({n1, n2});
	l->connect({n2, n3});
	sw->connect({EMT::SimNode::GND, n3});

	auto sys = SystemTopology(50,
		SystemNodeList{ n1, n2, n3 },
		SystemComponentList{ vs, r, l, sw });

	auto logger = CPS::DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v3", n3->attribute("v"));
	logger->logAttribute("i_r", r->attribute("i_intf"));

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setDomain(CPS::Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.addLogger(logger);

	sim.run();
}

int main(int argc, char* argv[]) {
	Real timeStep = 0.00005;
	Real finalTime = 0.2;
	Real voltageStep = 0.0;
	Real freqStep = 0.0;
	Real stepTime = 0.1;
	UInt maxTimeStepIdx = 20;
	UInt maxVoltageStepIdx = 10;
	UInt maxFreqStepIdx = 10;

	for (UInt stepIdx = 1; stepIdx <= maxTimeStepIdx; ++stepIdx) {
		timeStep = stepIdx * 0.00005;

		EMT_RL_SourceStep(timeStep, finalTime, 0, 0, stepTime, "_T" + std::to_string(stepIdx));
		DP_RL_SourceStep(timeStep, finalTime, 0, 0, stepTime, "_T" + std::to_string(stepIdx));

		for (UInt voltageStepIdx = 1; voltageStepIdx <= maxVoltageStepIdx; ++voltageStepIdx) {
			voltageStep = voltageStepIdx * 100;
			EMT_RL_SourceStep(timeStep, finalTime, voltageStep, 0, stepTime,
				"_T" + std::to_string(stepIdx) + "_V" + std::to_string(voltageStepIdx));
			DP_RL_SourceStep(timeStep, finalTime, voltageStep, 0, stepTime,
				"_T" + std::to_string(stepIdx) + "_V" + std::to_string(voltageStepIdx));
		}

		for (UInt freqStepIdx = 1; freqStepIdx <= maxFreqStepIdx; ++freqStepIdx) {
			freqStep = freqStepIdx;
			EMT_RL_SourceStep(timeStep, finalTime, 0, freqStep, stepTime,
				"_T" + std::to_string(stepIdx) + "_F" + std::to_string(freqStepIdx));
			DP_RL_SourceStep(timeStep, finalTime, 0, freqStep, stepTime,
				"_T" + std::to_string(stepIdx) + "_F" + std::to_string(freqStepIdx));
		}
	}

	for (UInt stepIdx = 40; stepIdx <= 400; stepIdx = stepIdx+20) {
		timeStep = stepIdx * 0.00005;

		EMT_RL_SourceStep(timeStep, finalTime, 0, 0, stepTime, "_T" + std::to_string(stepIdx));
		DP_RL_SourceStep(timeStep, finalTime, 0, 0, stepTime, "_T" + std::to_string(stepIdx));

		for (UInt voltageStepIdx = 1; voltageStepIdx <= maxVoltageStepIdx; ++voltageStepIdx) {
			voltageStep = voltageStepIdx * 100;
			EMT_RL_SourceStep(timeStep, finalTime, voltageStep, 0, stepTime,
				"_T" + std::to_string(stepIdx) + "_V" + std::to_string(voltageStepIdx));
			DP_RL_SourceStep(timeStep, finalTime, voltageStep, 0, stepTime,
				"_T" + std::to_string(stepIdx) + "_V" + std::to_string(voltageStepIdx));
		}

		for (UInt freqStepIdx = 1; freqStepIdx <= maxFreqStepIdx; ++freqStepIdx) {
			freqStep = freqStepIdx;
			EMT_RL_SourceStep(timeStep, finalTime, 0, freqStep, stepTime,
				"_T" + std::to_string(stepIdx) + "_F" + std::to_string(freqStepIdx));
			DP_RL_SourceStep(timeStep, finalTime, 0, freqStep, stepTime,
				"_T" + std::to_string(stepIdx) + "_F" + std::to_string(freqStepIdx));
		}
	}
}
