/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;

int main(int argc, char* argv[]) {

	CIM::Examples::Grids::SGIB::ScenarioConfig scenario;

	Real finalTime = 2;
	Real timeStep = 0.0001;
	String simName = "DP_Slack_PiLine_VSI_with_PF_Init";
	Bool pvWithControl = true;
	Real cmdScaleP = 1.0;
	Real cmdScaleI = 1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;

		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("control") != args.options.end())
			pvWithControl = args.getOptionBool("control");
		if (args.options.find("scale_kp") != args.options.end())
			cmdScaleI = args.getOptionReal("scale_kp");
		if (args.options.find("scale_ki") != args.options.end())
			cmdScaleI = args.getOptionReal("scale_ki");
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName+"_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
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
	auto loggerPF = CPS::DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = simName+"_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetDP->setParameters(Complex(scenario.systemNominalVoltage,0));

	auto lineDP = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineDP->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance);


	auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv", "pv", Logger::Level::debug, true);
	pv->setParameters(scenario.systemOmega, scenario.pvNominalVoltage, scenario.pvNominalActivePower, scenario.pvNominalReactivePower);
	pv->setControllerParameters(cmdScaleP*scenario.KpPLL, cmdScaleI*scenario.KiPLL, cmdScaleP*scenario.KpPowerCtrl, cmdScaleI*scenario.KiPowerCtrl, cmdScaleP*scenario.KpCurrCtrl, cmdScaleI*scenario.KiCurrCtrl, scenario.OmegaCutoff);
	pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
	pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvNominalVoltage, 0, 0, scenario.transformerInductance);
	pv->setInitialStateValues(scenario.pvNominalActivePower, scenario.pvNominalReactivePower, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
	pv->withControl(pvWithControl);

	// Topology
	extnetDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	pv->connect({ n2DP });
	auto systemDP = SystemTopology(50,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{extnetDP, lineDP, pv});

	// Initialization of dynamic topology with values from powerflow
	systemDP.initWithPowerflow(systemPF);

	// Logging
	auto loggerDP = CPS::DataLogger::make(simNameDP);
	loggerDP->logAttribute("v1", n1DP->attribute("v"));
	loggerDP->logAttribute("v2", n2DP->attribute("v"));
	loggerDP->logAttribute("i12", lineDP->attribute("i_intf"));

	CIM::Examples::Grids::CIGREMV::logPVAttributes(loggerDP, pv);

	// load step sized in absolute terms
	// std::shared_ptr<SwitchEvent> loadStepEvent = CIM::Examples::Events::createEventAddPowerConsumption("n2", std::round(5.0/timeStep)*timeStep, 10e6, systemDP, Domain::DP, loggerDP);

	// Simulation
	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);
	// sim.addEvent(loadStepEvent);
	sim.addLogger(loggerDP);
	sim.run();

}
