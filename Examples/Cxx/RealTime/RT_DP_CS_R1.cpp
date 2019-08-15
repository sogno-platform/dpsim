/** Reference Circuits
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
using namespace CPS::DP::Ph1;

	void RT_DP_CS_R1();
	void Ref_DP_CS_R1();

int main(int argc, char* argv[]) {
	RT_DP_CS_R1();
	Ref_DP_CS_R1();
}

void RT_DP_CS_R1() {
	Real timeStep = 0.001;
	Real finalTime = 5;
	String simName = "RT_DP_CS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);

	// Connections
	cs->connect({ Node::GND, n1 });
	r1->connect({ Node::GND, n1 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{Node::GND, n1},
		SystemComponentList{cs, r1});

	RealTimeSimulation sim(simName, Logger::Level::off);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);

	sim.run();
	sim.logStepTimes(simName + "_step_times");
}

void Ref_DP_CS_R1() {
	Real timeStep = 0.001;
	Real finalTime = 5;
	String simName = "Ref_DP_CS_R1";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = Node::make("n1");

	// Components
	auto cs = CurrentSource::make("cs");
	cs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);

	// Connections
	cs->connect({ Node::GND, n1 });
	r1->connect({ Node::GND, n1 });

	// Define system topology
	auto sys = SystemTopology(50,
		SystemNodeList{Node::GND, n1},
		SystemComponentList{cs, r1});

	Simulation sim(simName, Logger::Level::off);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);

	sim.run();
	sim.logStepTimes(simName + "_step_times");
}
