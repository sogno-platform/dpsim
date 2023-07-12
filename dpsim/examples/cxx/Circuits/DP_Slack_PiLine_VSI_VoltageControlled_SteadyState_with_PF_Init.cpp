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

	Real finalTime = 0.5;
	Real timeStep = 0.0001;
	String simName = "DP_Slack_PiLine_VSI_VoltageControlled_SteadyState_with_PF_Init";
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
	linePF->setParameters(0.88e-3, 0, 0, 0);
	linePF->setBaseVoltage(400);

	Complex load1_s=3*std::pow(400, 2)/(Complex(83e-3, 137e-6*2*M_PI*60));
	Real load1_p=load1_s.real();
	Real load1_q=load1_s.imag();

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(load1_p, load1_q, 400);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ);

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
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



	// ----- DP SIMULATION -----
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	
	auto loadDP = DP::Ph1::RXLoad::make("Load", Logger::Level::debug);
	loadDP->setParameters(load1_p, load1_q, 400);

	auto pv = DP::Ph1::VSIVoltageControlDQ::make("pv", "pv", Logger::Level::debug, false);
	pv->setParameters(Yazdani.OmegaNull, Yazdani.Vdref, Yazdani.Vqref);
	pv->setControllerParameters(Yazdani.KpVoltageCtrl, Yazdani.KiVoltageCtrl, Yazdani.KpCurrCtrl, Yazdani.KiCurrCtrl, Yazdani.KpPLL, Yazdani.KiPLL, Yazdani.OmegaCutoff);
	pv->setFilterParameters(Yazdani.Lf, Yazdani.Cf, Yazdani.Rf, Yazdani.Rc); 
	pv->setInitialStateValues(Yazdani.phi_dInit, Yazdani.phi_qInit, Yazdani.gamma_dInit, Yazdani.gamma_qInit);
	pv->withControl(pvWithControl);

	auto resOnDP = DP::Ph1::Resistor::make("R2", Logger::Level::debug);
	resOnDP->setParameters(0.88e-3);

	// Topology
	pv->connect({ n1DP });
	resOnDP->connect({n1DP, n2DP});
	loadDP->connect({n2DP});

	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{loadDP, resOnDP, pv});

	// Initialization of dynamic topology
	systemDP.initWithPowerflow(systemPF);
	Complex initial1PhPowerVSI= Complex(linePF->attributeTyped<Real>("p_inj")->get(), linePF->attributeTyped<Real>("q_inj")->get());

	pv->terminal(0)->setPower(initial1PhPowerVSI);

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->logAttribute("Spannung_PCC", n1DP->attribute("v"));
    loggerDP->logAttribute("Spannung_Quelle", pv->attribute("Vs"));
	loggerDP->logAttribute("Strom_RLC", pv->attribute("i_intf"));
	loggerDP->logAttribute("PLL_Phase", pv->attribute("pll_output"));
	loggerDP->logAttribute("P_elec", pv->attribute("P_elec"));
	loggerDP->logAttribute("Q_elec", pv->attribute("Q_elec"));
	
	// Simulation
	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);
	sim.addLogger(loggerDP);
	sim.run();
}