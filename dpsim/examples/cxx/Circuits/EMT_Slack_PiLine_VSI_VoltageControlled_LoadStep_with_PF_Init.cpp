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

	CIM::Examples::Grids::GridForming::ScenarioConfig1 scenario;
	// CIM::Examples::Grids::GridForming::Yazdani scenario;

	Real finalTime = 5;
	Real timeStep = 0.0001;
	String simName = "EMT_Slack_PiLine_VSI_VoltageControlled_LoadStep_with_PF_Init";
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
	linePF->setParameters(scenario.lineResistance, scenario.lineInductance, scenario.lineCapacitance, scenario.lineConductance);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	auto systemPF = SystemTopology(scenario.systemNominalFreq,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{linePF, extnetPF});
	
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

	// Components EMT Simulation
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);
	auto n4EMT = SimNode<Real>::make("n4", PhaseType::ABC);

	Complex load1_s=3*std::pow(scenario.systemNominalVoltage, 2)/(Complex(scenario.loadRes1, scenario.loadInd1*scenario.systemNominalOmega));
	Real load1ActivePower=load1_s.real();
	Real load1ReactivePower=load1_s.imag();

	Complex load2_s=3*std::pow(scenario.systemNominalVoltage, 2)/(Complex(scenario.loadRes2, scenario.loadInd2*scenario.systemNominalOmega - (1/(scenario.systemNominalOmega*scenario.loadCap2)) ));
	Real load2ActivePower=load2_s.real();
	Real load2ReactivePower=load2_s.imag();
	
	auto load1EMT = EMT::Ph3::RXLoad::make("Load1", Logger::Level::debug);
	load1EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(load1ActivePower), CPS::Math::singlePhasePowerToThreePhase(load1ReactivePower), scenario.systemNominalVoltage);

	auto load2EMT = EMT::Ph3::RXLoad::make("Load2", Logger::Level::debug);
	load2EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(load2ActivePower), CPS::Math::singlePhasePowerToThreePhase(load2ReactivePower), scenario.systemNominalVoltage);
	
	auto pv = EMT::Ph3::VSIVoltageControlDQ::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(scenario.systemNominalOmega, scenario.Vdref, scenario.Vqref, scenario.Pref); //initialise with Pref
	pv->setControllerParameters(scenario.KpVoltageCtrl, scenario.KiVoltageCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.systemNominalOmega, scenario.m_p, scenario.tau_p, scenario.tau_l); //Initialise with taup taui mp
	pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
	pv->setInitialStateValues(scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);
	pv->withControl(pvWithControl);


	auto lineEMT = EMT::Ph3::Resistor::make("Line", Logger::Level::debug);
	lineEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(scenario.lineResistance));

	// Fault Event
    Real SwitchOpen = 1e9;
    Real SwitchClosed = 1e-9;

    auto load1SwitchEMT = CPS::EMT::Ph3::Switch::make("Switch1", Logger::Level::debug);
	load1SwitchEMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	load1SwitchEMT->openSwitch();

    auto load2SwitchEMT = CPS::EMT::Ph3::Switch::make("Switch2", Logger::Level::debug);
	load2SwitchEMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	load2SwitchEMT->openSwitch();

	// Topology
	pv->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
    load1SwitchEMT-> connect({n2EMT, n3EMT});
	load1EMT->connect({n3EMT});
    load2SwitchEMT->connect({n3EMT, n4EMT});
	load2EMT->connect({n4EMT});
	
	auto systemEMT = SystemTopology(scenario.systemNominalFreq,
			SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT},
			SystemComponentList{pv,lineEMT, load1SwitchEMT, load1EMT, load2SwitchEMT, load2EMT});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF, CPS::Domain::EMT);

	Complex initial3PhPowerVSI = Complex(linePF->attributeTyped<Real>("p_inj")->get(), linePF->attributeTyped<Real>("q_inj")->get());

	pv->terminal(0)->setPower(initial3PhPowerVSI);

	// Logging

	
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("Spannung_PCC", n2EMT->attribute("v"));
	loggerEMT->logAttribute("Spannung_Load1", n3EMT->attribute("v"));
	loggerEMT->logAttribute("Spannung_Load2", n4EMT->attribute("v"));
    loggerEMT->logAttribute("Spannung_Quelle", pv->attribute("Vs"));
	loggerEMT->logAttribute("Strom_RLC", pv->attribute("i_intf"));
	loggerEMT->logAttribute("Droop_Output", pv->attribute("droop_output"));
	//loggerEMT->logAttribute("PLL_Phase", pv->attribute("pll_output"));
	loggerEMT->logAttribute("VCO_Phase", pv->attribute("vco_output"));
	loggerEMT->logAttribute("P_elec", pv->attribute("P_elec"));
	loggerEMT->logAttribute("Q_elec", pv->attribute("Q_elec"));
	

	

	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);
	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.addLogger(loggerEMT);

	// Events
    Real startTimeLoad1Step = 1;
    Real startTimeLoad2Step = 2;
    Real endTimeLoad1Step = 3;
	Real endTimeLoad2Step = 4;

    auto sw1close = SwitchEvent3Ph::make(startTimeLoad1Step, load1SwitchEMT, true);
	sim.addEvent(sw1close);
    auto sw2close = SwitchEvent3Ph::make(startTimeLoad2Step, load2SwitchEMT, true);
	sim.addEvent(sw2close);
	auto sw1open = SwitchEvent3Ph::make(endTimeLoad1Step, load1SwitchEMT, false);
	sim.addEvent(sw1open);
	auto sw2open = SwitchEvent3Ph::make(endTimeLoad2Step, load2SwitchEMT, false);
	sim.addEvent(sw2open);

	sim.run();
}
