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

	// CIM::Examples::Grids::GridForming::ScenarioConfig1 scenario;
	CIM::Examples::Grids::GridForming::Yazdani scenario;

	Real finalTime = 2;
	Real timeStep = 0.0001;
	String simName = "EMT_Slack_PiLine_VSI_VoltageControlled_SteadyState_with_PF_Init";
	Bool pvWithControl = true;


	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);
	
	// Components Powerflow Init
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(scenario.systemNominalVoltage);
	extnetPF->setBaseVoltage(scenario.systemNominalVoltage);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(scenario.lineResistance, 0, 0, 0);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	Complex load1_s=3*std::pow(scenario.systemNominalVoltage, 2)/(Complex(scenario.loadRes1, scenario.loadInd1*scenario.systemNominalOmega));
	Real loadActivePower=load1_s.real();
	Real loadReactivePower=load1_s.imag();

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(loadActivePower, loadReactivePower, scenario.systemNominalVoltage);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	// auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	// loadPF->setParameters(scenario.loadActivePower, scenario.loadReactivePower, scenario.systemNominalVoltage);
	// loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });
	auto systemPF = SystemTopology(scenario.systemNominalFreq,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{linePF, extnetPF, loadPF});
	
	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
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



	// ----- EMT SIMULATION -----
	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);

	// Components
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	
	auto loadEMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(loadActivePower), CPS::Math::singlePhasePowerToThreePhase(loadReactivePower), scenario.systemNominalVoltage);

	auto pv = EMT::Ph3::VSIVoltageControlDQ::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(scenario.systemNominalOmega, scenario.Vdref, scenario.Vqref);
	pv->setControllerParameters(scenario.KpVoltageCtrl, scenario.KiVoltageCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.systemNominalOmega);
	pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
	pv->setInitialStateValues(scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
	pv->withControl(pvWithControl);

	// auto lineEMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	// lineEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(scenario.lineResistance), CPS::Math::singlePhaseParameterToThreePhase(scenario.lineInductance), 0, 0);

	auto lineEMT = EMT::Ph3::Resistor::make("Line", Logger::Level::debug);
	lineEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(scenario.lineResistance));

	// Topology
	pv->connect({ n1EMT });
	lineEMT->connect({n1EMT, n2EMT});
	loadEMT->connect({n2EMT});

	auto systemEMT = SystemTopology(scenario.systemNominalFreq,
		SystemNodeList{n1EMT, n2EMT},
		SystemComponentList{loadEMT, lineEMT, pv});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF);
	Complex initial3PhPowerVSI= Complex(linePF->attributeTyped<Real>("p_inj")->get(), linePF->attributeTyped<Real>("q_inj")->get());

	pv->terminal(0)->setPower(initial3PhPowerVSI);

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("Controlled_source_PV", pv->attribute("Vs"));
	loggerEMT->logAttribute("Voltage_terminal_PV", n1EMT->attribute("v"));
	loggerEMT->logAttribute("Voltage_PCC", n2EMT->attribute("v"));
	loggerEMT->logAttribute("Strom_PV", pv->attribute("i_intf"));
	loggerEMT->logAttribute("VCO_output", pv->attribute("vco_output"));
	loggerEMT->logAttribute("P_elec", pv->attribute("P_elec"));
	loggerEMT->logAttribute("Q_elec", pv->attribute("Q_elec"));
	
	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.addLogger(loggerEMT);
	sim.run();
}