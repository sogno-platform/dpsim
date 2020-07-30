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

int main(int argc, char* argv[]) {
	
	Real timeStep = 0.001;
	Real finalTime = 0.001;
	String simName = "PF_Slack_PiLine_PQLoad";
	Logger::setLogDir("logs/" + simName);

	// Parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;

	// Components
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
	
	auto extnetPF = SP::Ph1::externalGridInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vnom);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, 0);
	linePF->setBaseVoltage(Vnom);

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(pLoadNom, qLoadNom, Vnom);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ); 

	// Topology
	extnetPF->connect({ n1 });
	linePF->connect({ n1, n2 });
	loadPF->connect({ n2 });	

	auto systemPF = SystemTopology(50,
			SystemNodeList{n1, n2},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));

	// Simulation
	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(systemPF);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.setSolverType(Solver::Type::NRP);
	sim.doPowerFlowInit(false);
	sim.addLogger(logger);
	sim.run();
}
