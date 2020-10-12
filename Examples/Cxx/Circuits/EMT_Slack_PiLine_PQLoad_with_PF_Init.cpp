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
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char* argv[]) {
	
	// Parameters
	Real Vnom = 20e3;
	Real pLoadNom = 100e3;
	Real qLoadNom = 50e3;
	Real lineResistance = 0.05;
	Real lineInductance = 0.1;
	Real lineCapacitance = 0.1e-6;
	Real finalTime = 2.0;

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = 2.0;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "EMT_Slack_PiLine_PQLoad_with_PF_Init_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::externalGridInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vnom);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance);
	linePF->setBaseVoltage(Vnom);

	auto loadPF = SP::Ph1::Shunt::make("Load", Logger::Level::debug);
	loadPF->setParameters(pLoadNom / std::pow(Vnom, 2), - qLoadNom / std::pow(Vnom, 2));
	loadPF->setBaseVoltage(Vnom);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });	
	auto systemPF = SystemTopology(50,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doPowerFlowInit(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Real timeStepEMT = 0.0001;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = "EMT_Slack_PiLine_PQLoad_with_PF_Init_EMT";
	Logger::setLogDir("logs/" + simNameEMT);

	// Components
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);

	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);

	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	lineEMT->setParameters(Reader::singlePhaseParameterToThreePhase(lineResistance), Reader::singlePhaseParameterToThreePhase(lineInductance), Reader::singlePhaseParameterToThreePhase(lineCapacitance));

	auto loadEMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT->setParameters(Reader::singlePhasePowerToThreePhase(pLoadNom), Reader::singlePhasePowerToThreePhase(qLoadNom), Vnom);

	// Topology
	extnetEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	loadEMT->connect({ n2EMT });	
	auto systemEMT = SystemTopology(50,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{extnetEMT, lineEMT, loadEMT});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameEMT, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemEMT);			

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->addAttribute("v1", n1EMT->attribute("v"));
	loggerEMT->addAttribute("v2", n2EMT->attribute("v"));

	// load step sized in absolute terms
	// std::shared_ptr<SwitchEvent> loadStepEvent = Examples::createEventAddPowerConsumption("n2", 1-timeStepEMT, 100e3, systemEMT, Domain::EMT, loggerEMT);

	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.doPowerFlowInit(false);
	sim.addLogger(loggerEMT);
	// sim.addEvent(loadStepEvent);
	sim.run();
	
}
