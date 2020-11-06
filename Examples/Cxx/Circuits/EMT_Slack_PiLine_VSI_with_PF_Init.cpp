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

	Examples::SGIB::ScenarioConfig scenario;
	
	Real finalTime = 2;
	Real timeStep = 0.0001;
	String simName = "EMT_Slack_PiLine_VSI_with_PF_Init";
	Bool pvWithControl = true;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		simName = args.name;
		timeStep = args.timeStep;
		finalTime = args.duration;

		if (args.options_bool.find("control") != args.options_bool.end())
			pvWithControl = args.options_bool["control"];
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::externalGridInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(scenario.systemNominalVoltage);
	extnetPF->setBaseVoltage(scenario.systemNominalVoltage);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(-scenario.pvNominalActivePower, -scenario.pvNominalReactivePower, scenario.systemNominalVoltage);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ); 

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
	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);

	// Components
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);

	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);

	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	lineEMT->setParameters(Reader::singlePhaseParameterToThreePhase(scenario.lineResistance), Reader::singlePhaseParameterToThreePhase(scenario.lineInductance), Reader::singlePhaseParameterToThreePhase(scenario.lineCapacitance));

	auto pv = EMT::Ph3::AvVoltageSourceInverterDQ::make("pv", "pv", Logger::Level::debug, true);
	pv->setParameters(scenario.systemOmega, scenario.pvNominalVoltage, scenario.pvNominalActivePower, scenario.pvNominalReactivePower);
	pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
	pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
	pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
	pv->setInitialStateValues(scenario.pvNominalActivePower, scenario.pvNominalReactivePower, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
	pv->withControl(pvWithControl);

	// Topology
	extnetEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	pv->connect({ n2EMT });	
	auto systemEMT = SystemTopology(50,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{extnetEMT, lineEMT, pv});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameEMT, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemEMT);			

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->addAttribute("v1", n1EMT->attribute("v"));
	loggerEMT->addAttribute("v2", n2EMT->attribute("v"));
	loggerEMT->addAttribute("i12", lineEMT->attribute("i_intf"));

	Examples::CIGREMV::logPVDecomposedAttributes(loggerEMT, pv);

	// load step sized in absolute terms
	std::shared_ptr<SwitchEvent3Ph> loadStepEvent = Examples::createEventAddPowerConsumption3Ph("n2", 3-timeStepEMT, 1000e3, systemEMT, Domain::EMT, loggerEMT);

	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.addLogger(loggerEMT);
	sim.addEvent(loadStepEvent);
	sim.run();
}