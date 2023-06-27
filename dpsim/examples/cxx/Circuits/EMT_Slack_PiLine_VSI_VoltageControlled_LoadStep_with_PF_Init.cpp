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

	CIM::Examples::Grids::SGIB::Yazdani Yazdani;

	Real finalTime = 1.0;
	Real timeStep = 0.0001;
	String simName = "EMT_Slack_PiLine_VSI_VoltageControlled_LoadStep_with_PF_Init";
	Bool pvWithControl = true;


	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

    /*
	// Components Powerflow Init
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(400);
	extnetPF->setBaseVoltage(400);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(0.88e-3, 0, 0);
	linePF->setBaseVoltage(400);

	Complex load1_s=3*std::pow(400, 2)/(Complex(83e-3, 137e-6*2*M_PI*60));
	Real load1_p=load1_s.real();
	Real load1_q=load1_s.imag();

	auto load1PF = SP::Ph1::Load::make("Load1", Logger::Level::debug);
	load1PF->setParameters(load1_p, load1_q , 400);
	load1PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	load1PF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{linePF, extnetPF, load1PF});

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
	simPF.run(); */

	/*
	// Components Powerflow Init
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);
	auto n4PF = SimNode<Complex>::make("n4", PhaseType::Single);
	
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(400);
	extnetPF->setBaseVoltage(400);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(0.88e-3, 0, 0);
	linePF->setBaseVoltage(400);	

	Complex load1_s=3*std::pow(400, 2)/(Complex(83e-3, 137e-6*2*M_PI*60));
	Real load1_p=load1_s.real();
	Real load1_q=load1_s.imag();

	Complex load2_s=3*std::pow(400, 2)/(Complex(Yazdani.Res2, Yazdani.Ind2*2*M_PI*60 - 1/(2*M_PI*60*Yazdani.Cap2)));
	Real load2_p=load2_s.real();
	Real load2_q=load2_s.imag();

	auto load1PF = SP::Ph1::Load::make("Load1", Logger::Level::debug);
	load1PF->setParameters(load1_p, load1_q , 0);
	load1PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	auto load2PF = SP::Ph1::Load::make("Load1", Logger::Level::debug);
	load2PF->setParameters(load2_p, load2_q , 0);
	load2PF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	auto switch1PF = SP::Ph1::PiLine::make("Switch_1", Logger::Level::debug);
	switch1PF->setParameters(1e9, 0, 0);
	switch1PF->setBaseVoltage(0);	

	auto switch2PF = SP::Ph1::PiLine::make("Switch_2", Logger::Level::debug);
	switch2PF->setParameters(1e9, 0, 0);
	switch2PF->setBaseVoltage(0);	


	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	switch1PF->connect({ n2PF, n3PF });
	switch2PF->connect({ n3PF, n4PF });
	load1PF->connect({ n3PF });
	load2PF->connect({ n4PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF, n4PF},
			SystemComponentList{linePF, extnetPF, load1PF, load2PF, switch1PF, switch2PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("v3", n3PF->attribute("v"));
	loggerPF->logAttribute("v4", n4PF->attribute("v"));

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
	*/

	// Components Powerflow Init
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	Complex load1_s=3*std::pow(400, 2)/(Complex(83e-3, 137e-6*2*M_PI*60));
	Real load1_p=load1_s.real();
	Real load1_q=load1_s.imag();

	Complex load2_s=3*std::pow(400, 2)/(Complex(Yazdani.Res2, Yazdani.Ind2*2*M_PI*60 - 1/(2*M_PI*60*Yazdani.Cap2)));
	Real load2_p=load2_s.real();
	Real load2_q=load2_s.imag();

	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(400);
	extnetPF->setBaseVoltage(400);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(0.88e-3, 0, 0);
	linePF->setBaseVoltage(400);	


	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	
	auto systemPF = SystemTopology(60,
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

	// ----- DYNAMIC EMT SIMULATION -----
	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);


	// Components External Network

	/*
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);
	auto n4EMT = SimNode<Real>::make("n4", PhaseType::ABC);
	auto n5EMT = SimNode<Real>::make("n5", PhaseType::ABC);
	auto n6EMT = SimNode<Real>::make("n6", PhaseType::ABC);
	auto n7EMT = SimNode<Real>::make("n6", PhaseType::ABC);

	auto resOnEMT = EMT::Ph3::Resistor::make("ResOn", Logger::Level::debug);
	resOnEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(0.88e-3));

	auto res1EMT = EMT::Ph3::Resistor::make("R1", Logger::Level::debug);
	res1EMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(Yazdani.Res1));

	auto res2EMT = EMT::Ph3::Resistor::make("R2", Logger::Level::debug);
	res2EMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(Yazdani.Res2));

	auto ind1EMT = EMT::Ph3::Inductor::make("L1", Logger::Level::debug);
	ind1EMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(Yazdani.Ind1));

	auto ind2EMT = EMT::Ph3::Inductor::make("L2", Logger::Level::debug);
	ind2EMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(Yazdani.Ind2));

	auto capEMT = EMT::Ph3::Capacitor::make("C2", Logger::Level::debug);
	capEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(Yazdani.Cap2));

	
	auto pv = EMT::Ph3::VSIVoltageControlDQ::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
	pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl, Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl, Yazdani.KpPLL, Yazdani.KiPLL, Yazdani.OmegaCutoff);
	pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc);
	pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit, Yazdani.gamma_dInit, Yazdani.gamma_qInit);
	pv->withControl(pvWithControl);
		
	
	auto pv = EMT::Ph3::VSIVoltageControlVCO::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
	pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl, Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl, Yazdani.OmegaNull);
	pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc); 
	pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit, Yazdani.gamma_dInit, Yazdani.gamma_qInit);
	pv->withControl(pvWithControl);
	

	// Fault Event
    Real SwitchOpen = 1e9;
    Real SwitchClosed = 1e-9;

    auto fault1EMT = CPS::EMT::Ph3::Switch::make("Switch1", Logger::Level::debug);
	fault1EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	fault1EMT->openSwitch();

    auto fault2EMT = CPS::EMT::Ph3::Switch::make("Switch2", Logger::Level::debug);
	fault2EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	fault2EMT->openSwitch();

	// Topology
	pv->connect({ n1EMT });
	resOnEMT->connect({ n1EMT, n2EMT });
    fault1EMT-> connect({n2EMT, n3EMT});
	res1EMT->connect({n3EMT, n4EMT});
	ind1EMT->connect({n4EMT, EMT::SimNode::GND});
    fault2EMT->connect({n3EMT, n5EMT});
	res2EMT->connect({n5EMT, n6EMT});
	ind2EMT->connect({n6EMT, n7EMT});
	capEMT->connect({n7EMT, EMT::SimNode::GND});
	
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT, n5EMT, n6EMT, n7EMT},
			SystemComponentList{resOnEMT,res1EMT, res2EMT, fault1EMT, fault2EMT, ind1EMT, ind2EMT, capEMT, pv});

	*/


	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);
	auto n4EMT = SimNode<Real>::make("n4", PhaseType::ABC);


	auto loadEMT1 = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT1->setParameters(CPS::Math::singlePhasePowerToThreePhase(load1_p), CPS::Math::singlePhasePowerToThreePhase(load1_q), 400);

	auto loadEMT2 = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT2->setParameters(CPS::Math::singlePhasePowerToThreePhase(load2_p), CPS::Math::singlePhasePowerToThreePhase(load2_q), 400);

	auto resOnEMT = EMT::Ph3::Resistor::make("ResOn", Logger::Level::debug);
	resOnEMT->setParameters(CPS::Math::singlePhaseParameterToThreePhase(0.88e-3));

	
	auto pv = EMT::Ph3::VSIVoltageControlVCO::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
	pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl, Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl, Yazdani.OmegaNull);
	pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc); 
	pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit, Yazdani.gamma_dInit, Yazdani.gamma_qInit);
	pv->withControl(pvWithControl);
	
	/*
	auto pv = EMT::Ph3::VSIVoltageControlDQ::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
	pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl, Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl, Yazdani.KpPLL, Yazdani.KiPLL, Yazdani.OmegaCutoff);
	pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc); 
	pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit, Yazdani.gamma_dInit, Yazdani.gamma_qInit);
	pv->withControl(pvWithControl);
	*/

	// Fault Event
    Real SwitchOpen = 1e9;
    Real SwitchClosed = 1e-9;

    auto fault1EMT = CPS::EMT::Ph3::Switch::make("Switch1", Logger::Level::debug);
	fault1EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	fault1EMT->openSwitch();

    auto fault2EMT = CPS::EMT::Ph3::Switch::make("Switch2", Logger::Level::debug);
	fault2EMT->setParameters(Math::singlePhaseParameterToThreePhase(SwitchOpen),
							Math::singlePhaseParameterToThreePhase(SwitchClosed));
	fault2EMT->openSwitch();

	// Topology
	pv->connect({ n1EMT });
	resOnEMT->connect({ n1EMT, n2EMT });
    fault1EMT-> connect({n2EMT, n3EMT});
	loadEMT1->connect({n3EMT});
    fault2EMT->connect({n3EMT, n4EMT});
	loadEMT2->connect({n4EMT});
	
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT},
			SystemComponentList{resOnEMT,fault1EMT, fault2EMT, loadEMT1, loadEMT2, pv});


	// Initialization of dynamic topology
	// systemEMT.initWithPowerflow(systemPF);

	// Initialization of dynamic topology
	
	systemEMT.initWithPowerflow(systemPF);

	Complex initial3PhPowerVSI = Complex(linePF->attributeTyped<Real>("p_inj")->get(), linePF->attributeTyped<Real>("q_inj")->get());

	pv->terminal(0)->setPower(initial3PhPowerVSI);
	

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("Spannung_PCC", n1EMT->attribute("v"));
	loggerEMT->logAttribute("Spannung_Node_2", n2EMT->attribute("v"));
	loggerEMT->logAttribute("Spannung_Node_3", n3EMT->attribute("v"));
    loggerEMT->logAttribute("Spannung_Quelle", pv->attribute("Vs"));
	loggerEMT->logAttribute("Strom_RLC", pv->attribute("i_intf"));
	//loggerEMT->logAttribute("PLL_Phase", pv->attribute("pll_output"));
	//loggerEMT->logAttribute("VCO_Phase", pv->attribute("VCO_output"));
	loggerEMT->logAttribute("P_elec", pv->attribute("P_elec"));
	loggerEMT->logAttribute("Q_elec", pv->attribute("Q_elec"));

    // Fault Event Timers
    Real startTimeFault1 = 0.2;
    Real endTimeFault1 = 0.8;
    Real startTimeFault2 = 0.4;
    Real endTimeFault2 = 0.6;

	// Simulation
	Simulation sim(simNameEMT, Logger::Level::debug);	

    auto sw1close = SwitchEvent3Ph::make(startTimeFault1, fault1EMT, true);
	sim.addEvent(sw1close);
	auto sw1open = SwitchEvent3Ph::make(endTimeFault1, fault1EMT, false);
	sim.addEvent(sw1open);

    auto sw2close = SwitchEvent3Ph::make(startTimeFault2, fault2EMT, true);
	sim.addEvent(sw2close);
	auto sw2open = SwitchEvent3Ph::make(endTimeFault2, fault2EMT, false);
	sim.addEvent(sw2open);

	sim.setSystem(systemEMT);
	sim.setTimeStep(timeStepEMT);
	sim.setFinalTime(finalTimeEMT);
	sim.setDomain(Domain::EMT);
	sim.addLogger(loggerEMT);
	sim.run();
}
