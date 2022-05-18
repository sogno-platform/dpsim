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
#include <dpsim-villas/InterfaceShmem.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph1;

int main(int argc, char *argv[]) {
	Real timeStep = 0.001;
	Real finalTime = 10;
	String simName = "ShmemControllableSource";

	InterfaceShmem intf("/dpsim01", "/dpsim10");

	// Nodes
	auto n1 = SimNode::make("n1");

	// Components
	auto ecs = CurrentSource::make("v_intf");
	ecs->setParameters(Complex(10, 0));
	auto r1 = Resistor::make("r_1");
	r1->setParameters(1);

	ecs->connect({ SimNode::GND, n1 });
	r1->connect({ SimNode::GND, n1 });

	ecs->mCurrentRef->setReference(intf.importComplex(0));
	intf.exportComplex(ecs->mIntfVoltage->deriveCoeff<Complex>(0, 0), 0);

	auto sys = SystemTopology(50, SystemNodeList{n1}, SystemComponentList{ecs, r1});
	
	RealTimeSimulation sim(simName, CPS::Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.setSolverType(Solver::Type::MNA);

	sim.addInterface(&intf);
	sim.run();

	return 0;
}
