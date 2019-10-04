/*********************************************************************************
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
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
using namespace CPS;

static void VarFreqRxLineResLoad_DP(Real timeStep, Real finalTime, Real freqStep, Real stepTime) {
	String simName = "DP_RL_LoadStep_FreqStep_1_" + std::to_string(timeStep);
	Logger::setLogDir("logs/"+simName);

	auto n1 = DP::Node::make("n1");
	auto n2 = DP::Node::make("n2");
	auto n3 = DP::Node::make("n3");

	auto vs = std::make_shared<DP::Ph1::VoltageSourceRamp>("v_s");
	vs->setParameters(1000, 2000, 0, 2*PI*freqStep, stepTime, 0);
	auto r = DP::Ph1::Resistor::make("r_line");
	r->setParameters(1);
	auto l = DP::Ph1::Inductor::make("l_line");
	l->setParameters(0.2);
	auto sw = DP::Ph1::Resistor::make("r_load");
	sw->setParameters(100);

	vs->connect({DP::Node::GND, n1});
	r->connect({n1, n2});
	l->connect({n2, n3});
	sw->connect({DP::Node::GND, n3});

	auto sys = SystemTopology(50,
		SystemNodeList{ n1, n2, n3 },
		SystemComponentList{ vs, r, l });

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);

	sim.run();
}

static void VarFreqRxLineResLoad_EMT(Real timeStep, Real finalTime, Real freqStep, Real stepTime) {
	String simName = "EMT_RL_LoadStep_FreqStep_1_" + std::to_string(timeStep);
	Logger::setLogDir("logs/"+simName);

	auto n1 = EMT::Node::make("n1");
	auto n2 = EMT::Node::make("n2");
	auto n3 = EMT::Node::make("n3");

	auto vs = std::make_shared<EMT::Ph1::VoltageSourceRamp>("v_s");
	vs->setParameters(1000, 2000, 2*PI*50, 2*PI*freqStep, stepTime, 0);
	auto r = EMT::Ph1::Resistor::make("r_line");
	r->setParameters(1);
	auto l = EMT::Ph1::Inductor::make("l_line");
	l->setParameters(0.2);
	auto sw = EMT::Ph1::Resistor::make("r_load");
	sw->setParameters(100);

	vs->connect({EMT::Node::GND, n1});
	r->connect({n1, n2});
	l->connect({n2, n3});
	sw->connect({EMT::Node::GND, n3});

	auto sys = SystemTopology(50,
		SystemNodeList{ n1, n2, n3 },
		SystemComponentList{ vs, r, l });

	Simulation sim(simName);
	sim.setSystem(sys);
	sim.setDomain(CPS::Domain::EMT);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);

	sim.run();
}

int main(int argc, char* argv[]) {
	Real timeStep = 0.00005;
	Real finalTime = 0.4;
	Real freqStep = 0;
	Real stepTime = 0.2;

	VarFreqRxLineResLoad_EMT(timeStep, finalTime, freqStep, stepTime);
	VarFreqRxLineResLoad_DP(timeStep, finalTime, freqStep, stepTime);
}
