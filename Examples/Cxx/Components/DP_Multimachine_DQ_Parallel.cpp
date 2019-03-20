/** SynGenDPBalancedResLoad Example
 *
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
using namespace CPS::DP::Ph3;

double doSim(std::shared_ptr<Scheduler> scheduler, int generators) {
	// Define simulation parameters
	Real timeStep = 0.00005;
	Real finalTime = 0.3;
	String name = "DP_Multimachine_DQ";

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
	Real fieldVoltage = 7.0821;

	// Define grid parameters
	Real Rload = 0.96;
	Real Rline = 0.032;
	Real Rsnub = 25;

	std::vector<Complex> initVoltN1 = std::vector<Complex>({
		Complex(initTerminalVolt * cos(initVoltAngle), initTerminalVolt * sin(initVoltAngle)),
		Complex(initTerminalVolt * cos(initVoltAngle - 2 * PI / 3), initTerminalVolt * sin(initVoltAngle - 2 * PI / 3)),
		Complex(initTerminalVolt * cos(initVoltAngle + 2 * PI / 3), initTerminalVolt * sin(initVoltAngle + 2 * PI / 3)) });
	auto n_load = Node::make("n3", PhaseType::ABC);

	SystemNodeList nodes({n_load});
	SystemComponentList components;

	for (int i = 0; i < generators; i++) {
		auto node = Node::make("n" + std::to_string(i), PhaseType::ABC, initVoltN1);
		nodes.push_back(node);

		auto gen = Ph3::SynchronGeneratorDQODE::make("Gen" + std::to_string(i));
		gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
			Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
			initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, initMechPower);
		gen->connect({node});
		components.push_back(gen);

		auto line = Ph3::SeriesResistor::make("R_line" + std::to_string(i));
		line->setParameters(Rline);
		line->connect({node, n_load});
		components.push_back(line);

		auto snub = Ph3::SeriesResistor::make("R_snub" + std::to_string(i));
		snub->setParameters(Rsnub);
		snub->connect({node, Node::GND});
		components.push_back(snub);
	}

	auto load = Ph3::SeriesResistor::make("R_load");
	load->setParameters(Rload);
	load->connect({n_load, Node::GND});
	components.push_back(load);

	// System
	auto sys = SystemTopology(60, nodes, components);

	// Simulation
	Simulation sim(name, sys, timeStep, finalTime,
		Domain::DP, Solver::Type::MNA, Logger::Level::NONE);

	sim.setScheduler(scheduler);
	sim.run();

	Real tot = 0;
	for (auto meas : sim.stepTimes()) {
		tot += meas;
	}
	return tot / sim.stepTimes().size();
}

int main(int argc, char* argv[]) {
	int reps = 50;
	std::vector<int> threads_choices = {1};
	std::cout << "generators";
	for (auto threads : threads_choices) {
		std::cout << ", thread_level" << threads;
	}
	std::cout << std::endl;
	for (int generators = 1; generators < 20; generators++) {
		std::cout << generators;
		for (auto threads : threads_choices) {
			Real tot = 0;
			for (int rep = 0; rep < reps; rep++) {
				auto sched = std::make_shared<ThreadLevelScheduler>(threads);
				tot += doSim(sched, generators);
			}
			Real avg = tot / reps;
			std::cout << "," << avg;
		}
		std::cout << std::endl;
	}
}
