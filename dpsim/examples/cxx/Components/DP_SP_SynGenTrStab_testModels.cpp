/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

// Define machine parameters in per unit
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real nomFreq = 60;
Real nomFieldCurr = 1300;
Int poleNum = 2;
Real H = 3.7;
Real Rs = 0.003;
Real Ll = 0.15;
Real Lmd = 1.6599;
Real Lmq = 1.61;
Real Rfd = 0.0006;
Real Llfd = 0.1648;
Real Rkd = 0.0284;
Real Llkd = 0.1713;
Real Rkq1 = 0.0062;
Real Llkq1 = 0.7252;
Real Rkq2 = 0.0237;
Real Llkq2 = 0.125;

//PiLine parameters calculated from CIGRE Benchmark system (works with 1us)
//1oo km length 230kV
// Real lineResistance = 0.073;
// Real lineInductance = 0.518/377;
// Real lineCapacitance = 0.032/377;
// Real lineConductance =1e-2;


//
Real lineResistance = 0.073;
Real lineInductance = 0.518/377;
Real lineCapacitance = 0.032/377;
Real lineConductance =8e-3; //change inductance to allow bigger time steps and to stabilize simulation 8e-2(10us)
//Real lineConductance =1e-3;

// //PiLine parameters
// Real lineResistance = 1e-3;
// Real lineInductance = 6e-3/377;
// Real lineCapacitance = 50e-6;


//Breaker to trigger fault between the two lines
Real BreakerOpen = 1e3;
// Real BreakerClosed = 0.001;
Real BreakerClosed = 0.005;

// Parameters for powerflow initialization
Real Vnom = nomPhPhVoltRMS;
// Slack voltage: 24kV
Real Vslack = Vnom;
//Synchronous generator
Real ratedApparentPower=555e6;
Real ratedVoltage=Vnom;
Real setPointActivePower=300e6;
Real setPointVoltage=Vnom+0.05*Vnom;

void SP_1ph_SynGenTrStab_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault){

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = "SP_PFinit";
	Logger::setLogDir("logs/" + simNamePF);
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
	genPF->setParameters(ratedApparentPower, ratedVoltage, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(Vnom);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(Vnom);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("v_line", linePF->attribute("v_intf"));
	loggerPF->logAttribute("i_line", linePF->attribute("i_intf"));
	loggerPF->logAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->logAttribute("ig", genPF->attribute("i_intf"));

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

	// ----- Dynamic simulation ------
	String simName = "SP_1ph_SynGenTrStab_Fault";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

	// Components
	auto gen = CPS::SP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);

	//Grid bus as Slack
	auto extnet = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line
	auto line = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	line->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);

	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->open();

	// Topology
	gen->connect({ n1 });
	line->connect({ n1, n2 });
	extnet->connect({ n2 });
	fault->connect({CPS::SP::SimNode::GND, n1});
	auto system = SystemTopology(60,
			SystemNodeList{n1, n2},
			SystemComponentList{gen, line, fault, extnet});

	// Initialization of dynamic topology
	system.initWithPowerflow(systemPF);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line", line->attribute("v_intf"));
	logger->logAttribute("i_line", line->attribute("i_intf"));
	logger->logAttribute("v_gen", gen->attribute("v_intf"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("delta_r", gen->attribute("Ep_phase"));
	logger->logAttribute("Ep", gen->attribute("Ep_mag"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent::make(startTimeFault, fault, true);

		sim.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
		sim.addEvent(sw2);

	}

	sim.run();
}

void DP_1ph_SynGenTrStab_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault) {
	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "DP_PFinit";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
	genPF->setParameters(ratedApparentPower, ratedVoltage, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(Vnom);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(Vnom);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("v_line", linePF->attribute("v_intf"));
	loggerPF->logAttribute("i_line", linePF->attribute("i_intf"));
	loggerPF->logAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->logAttribute("ig", genPF->attribute("i_intf"));

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

	// ----- Dynamic simulation ------
	String simName = "DP_1ph_SynGenTrStab_Fault";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

	// Components
	auto gen = CPS::DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);

	//Grid bus as Slack
	auto extnet = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line
	auto line = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	line->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->open();

	// Topology
	gen->connect({ n1 });
	line->connect({ n1, n2 });
	extnet->connect({ n2 });
	fault->connect({CPS::DP::SimNode::GND, n1});
	auto system = SystemTopology(60,
			SystemNodeList{n1, n2},
			SystemComponentList{gen, line, fault, extnet});

	// Initialization of dynamic topology
	system.initWithPowerflow(systemPF);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line", line->attribute("v_intf"));
	logger->logAttribute("i_line", line->attribute("i_intf"));
	logger->logAttribute("v_gen", gen->attribute("v_intf"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("delta_r", gen->attribute("Ep_phase"));
	logger->logAttribute("Ep", gen->attribute("Ep_mag"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent::make(startTimeFault, fault, true);

		sim.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
		sim.addEvent(sw2);

	}

	sim.run();
}

int main(int argc, char* argv[]) {

	//Simultion parameters
	Real finalTime = 5;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	// Bool startFaultEvent= false;
	Bool endFaultEvent=false;
	Real startTimeFault=1;
	Real endTimeFault=2;

	//Transient Stability model(classical model 2nd order)
	SP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//Transient Stability model(classical model 2nd order)
	DP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//change file names DP_SP_...

}
