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
	simPF.run();



	// ----- DYNAMIC EMT SIMULATION -----
	Real timeStepEMT = timeStep;
	Real finalTimeEMT = finalTime+timeStepEMT;
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/" + simNameEMT);


	// Components External Network
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);
	auto n4EMT = SimNode<Real>::make("n4", PhaseType::ABC);
	auto n5EMT = SimNode<Real>::make("n5", PhaseType::ABC);
	auto n6EMT = SimNode<Real>::make("n6", PhaseType::ABC);

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
    fault1EMT-> connect({n1EMT, n2EMT});
	res1EMT->connect({n2EMT, n3EMT});
	ind1EMT->connect({n3EMT, EMT::SimNode::GND});
    fault2EMT->connect({n2EMT, n4EMT});
	res2EMT->connect({n4EMT, n5EMT});
	ind2EMT->connect({n5EMT, n6EMT});
	capEMT->connect({n6EMT, EMT::SimNode::GND});
	
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT, n3EMT, n4EMT, n5EMT, n6EMT},
			SystemComponentList{res1EMT, res2EMT, fault1EMT, fault2EMT, ind1EMT, ind2EMT, capEMT, pv});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF);

	// Initialization of dynamic topology
	/*systemEMT.initWithPowerflow(systemPF);

	Complex initial3PhPowerVSI= Complex(linePF->attributeTyped<Real>("p_inj")->get(), linePF->attributeTyped<Real>("q_inj")->get());

	pv->terminal(0)->setPower(initial3PhPowerVSI);
	*/

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->logAttribute("Spannung_PCC", n1EMT->attribute("v"));
    loggerEMT->logAttribute("Spannung_Quelle", pv->attribute("Vs"));
	loggerEMT->logAttribute("Strom_RLC", pv->attribute("i_intf"));
	loggerEMT->logAttribute("PLL_Phase", pv->attribute("pll_output"));
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
