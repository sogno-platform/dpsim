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
	String simName = "DP_Slack_PiLine_VSI_Ramp_with_PF_Init";
	Bool pvWithControl = true;
	Real cmdScaleP = 1.0;
	Real cmdScaleI = 1.0;

	Real rampRocof = -6.25;
	Real rampDuration = 0.4;

	String logDirectory = "logs";

	Logger::Level logLevel = Logger::Level::info;

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
		if (args.options.find("ramp_rocof") != args.options.end())
			rampRocof = args.getOptionReal("ramp_rocof");
		if (args.options.find("ramp_duration") != args.options.end())
			rampDuration = args.getOptionReal("ramp_duration");
		if (args.options.find("logDirectory") != args.options.end())
			logDirectory = args.getOptionString("logDirectory");
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName+"_PF";
	Logger::setLogDir(logDirectory + "/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetPF->setParameters(scenario.systemNominalVoltage);
	extnetPF->setBaseVoltage(scenario.systemNominalVoltage);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", logLevel);
	linePF->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	auto loadPF = SP::Ph1::Load::make("Load", logLevel);
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
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, logLevel);
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
	Logger::setLogDir(logDirectory + "/" + simName);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetDP->setParameters(Complex(scenario.systemNominalVoltage, 0), 0, rampRocof, 5.0, rampDuration);

	auto lineDP = DP::Ph1::PiLine::make("PiLine", logLevel);
	lineDP->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance);


	auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv", "pv", logLevel, true);
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
	systemDP.initWithPowerflow(systemPF, Domain::DP);

	// Logging
	auto loggerDP = DataLogger::make(simName);
	loggerDP->logAttribute("v1", n1DP->attribute("v"));
	loggerDP->logAttribute("v2", n2DP->attribute("v"));
	loggerDP->logAttribute("i12", lineDP->attribute("i_intf"));
	loggerDP->logAttribute("f_src", extnetDP->attribute("f_src"));
	loggerDP->logAttribute("pv_v_intf", pv->attribute("v_intf"));
    loggerDP->logAttribute("pv_i_intf", pv->attribute("i_intf"));

	// CIM::Examples::Grids::CIGREMV::logPVAttributes(loggerDP, pv);

	// Simulation
	Simulation sim(simName, logLevel);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);

	sim.addLogger(loggerDP);
	sim.run();

}
