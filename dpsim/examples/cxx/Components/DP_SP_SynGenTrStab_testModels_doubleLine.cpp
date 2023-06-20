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
Real H = 3.7/10;
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

//PiLine parameters calculated from CIGRE Benchmark system
Real lineResistance = 0.073;
Real lineInductance = 0.518/377;
Real lineCapacitance = 0.032/377;
Real lineConductance =8e-2; //change inductance to allow bigger time steps and to stabilize simulation 8e-2(10us)

// Fault in Line 2 at l=10km
Real lineResistance21 = lineResistance*0.1;
Real lineInductance21 = lineInductance*0.1;
Real lineCapacitance21 = lineCapacitance*0.1;
Real lineConductance21 =lineConductance*0.1;

Real lineResistance22 = lineResistance*0.9;
Real lineInductance22 = lineInductance*0.9;
Real lineCapacitance22 = lineCapacitance*0.9;
Real lineConductance22 =lineConductance*0.9;

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

	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "SP_PFinit_dl";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

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
	//Line1
	auto linePF1 = SP::Ph1::PiLine::make("PiLine1", Logger::Level::debug);
	linePF1->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF1->setBaseVoltage(Vnom);
	//Line21
	auto linePF21 = SP::Ph1::PiLine::make("PiLine21", Logger::Level::debug);
	linePF21->setParameters(lineResistance21, lineInductance21, lineCapacitance21, lineConductance21);
	linePF21->setBaseVoltage(Vnom);
	//Line22
	auto linePF22 = SP::Ph1::PiLine::make("PiLine22", Logger::Level::debug);
	linePF22->setParameters(lineResistance22, lineInductance22, lineCapacitance22, lineConductance22);
	linePF22->setBaseVoltage(Vnom);

	// Topology
	genPF->connect({ n1PF });
	linePF1->connect({ n1PF, n3PF });
	linePF21->connect({ n1PF, n2PF });
	linePF22->connect({ n2PF, n3PF });
	extnetPF->connect({ n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{genPF, linePF1, linePF21, linePF22, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("v_line1", linePF1->attribute("v_intf"));
	loggerPF->logAttribute("i_line1", linePF1->attribute("i_intf"));
	loggerPF->logAttribute("v_line21", linePF21->attribute("v_intf"));
	loggerPF->logAttribute("i_line21", linePF21->attribute("i_intf"));
	loggerPF->logAttribute("v_line22", linePF22->attribute("v_intf"));
	loggerPF->logAttribute("i_line22", linePF22->attribute("i_intf"));
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
	String simName = "SP_1ph_SynGenTrStab_Fault_dl";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	auto gen = CPS::SP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);

	//Grid bus as Slack
	auto extnet = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnet->setBaseVoltage(Vnom);

	// Line 1
	auto line1 = SP::Ph1::PiLine::make("PiLine1", Logger::Level::debug);
	line1->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	line1->setBaseVoltage(Vnom);
	//Line21
	auto line21 = SP::Ph1::PiLine::make("PiLine21", Logger::Level::debug);
	line21->setParameters(lineResistance21, lineInductance21, lineCapacitance21, lineConductance21);
	line21->setBaseVoltage(Vnom);
	//Line22
	auto line22 = SP::Ph1::PiLine::make("PiLine22", Logger::Level::debug);
	line22->setParameters(lineResistance22, lineInductance22, lineCapacitance22, lineConductance22);
	line22->setBaseVoltage(Vnom);
	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->open();

	// Topology
	gen->connect({ n1 });
	line1->connect({ n1, n3 });
	line21->connect({ n1, n2 });
	line22->connect({ n2, n3 });
	extnet->connect({ n3 });
	fault->connect({CPS::SP::SimNode::GND, n2});
	auto system = SystemTopology(60,
			SystemNodeList{n1, n2, n3},
			SystemComponentList{gen, line1, line21, fault, line22, extnet});



	// Initialization of dynamic topology
	system.initWithPowerflow(systemPF);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line1", line1->attribute("v_intf"));
	logger->logAttribute("i_line1", line1->attribute("i_intf"));
	logger->logAttribute("v_line21", line21->attribute("v_intf"));
	logger->logAttribute("i_line21", line21->attribute("i_intf"));
	logger->logAttribute("v_line22", line22->attribute("v_intf"));
	logger->logAttribute("i_line22", line22->attribute("i_intf"));
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
	String simNamePF = "DP_PFinit_dl";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

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
	//Line1
	auto linePF1 = SP::Ph1::PiLine::make("PiLine1", Logger::Level::debug);
	linePF1->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF1->setBaseVoltage(Vnom);
	//Line21
	auto linePF21 = SP::Ph1::PiLine::make("PiLine21", Logger::Level::debug);
	linePF21->setParameters(lineResistance21, lineInductance21, lineCapacitance21, lineConductance21);
	linePF21->setBaseVoltage(Vnom);
	//Line22
	auto linePF22 = SP::Ph1::PiLine::make("PiLine22", Logger::Level::debug);
	linePF22->setParameters(lineResistance22, lineInductance22, lineCapacitance22, lineConductance22);
	linePF22->setBaseVoltage(Vnom);

	// Topology
	genPF->connect({ n1PF });
	linePF1->connect({ n1PF, n3PF });
	linePF21->connect({ n1PF, n2PF });
	linePF22->connect({ n2PF, n3PF });
	extnetPF->connect({ n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{genPF, linePF1, linePF21, linePF22, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));
	loggerPF->logAttribute("v_line1", linePF1->attribute("v_intf"));
	loggerPF->logAttribute("i_line1", linePF1->attribute("i_intf"));
	loggerPF->logAttribute("v_line21", linePF21->attribute("v_intf"));
	loggerPF->logAttribute("i_line21", linePF21->attribute("i_intf"));
	loggerPF->logAttribute("v_line22", linePF22->attribute("v_intf"));
	loggerPF->logAttribute("i_line22", linePF22->attribute("i_intf"));
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
	String simName = "DP_1ph_SynGenTrStab_Fault_dl";
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	auto gen = CPS::DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);

	//Grid bus as Slack
	auto extnet = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line 1
	auto line1 = DP::Ph1::PiLine::make("PiLine1", Logger::Level::debug);
	line1->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	//Line21
	auto line21 = DP::Ph1::PiLine::make("PiLine21", Logger::Level::debug);
	line21->setParameters(lineResistance21, lineInductance21, lineCapacitance21, lineConductance21);
	//Line22
	auto line22 = DP::Ph1::PiLine::make("PiLine22", Logger::Level::debug);
	line22->setParameters(lineResistance22, lineInductance22, lineCapacitance22, lineConductance22);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->open();

	// Topology
	gen->connect({ n1 });
	line1->connect({ n1, n3 });
	line21->connect({ n1, n2 });
	line22->connect({ n2, n3 });
	extnet->connect({ n3 });
	fault->connect({CPS::DP::SimNode::GND, n2});
	auto system = SystemTopology(60,
			SystemNodeList{n1, n2, n3},
			SystemComponentList{gen, line1, line21, fault, line22, extnet});

	// Initialization of dynamic topology
	system.initWithPowerflow(systemPF);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line1", line1->attribute("v_intf"));
	logger->logAttribute("i_line1", line1->attribute("i_intf"));
	logger->logAttribute("v_line21", line21->attribute("v_intf"));
	logger->logAttribute("i_line21", line21->attribute("i_intf"));
	logger->logAttribute("v_line22", line22->attribute("v_intf"));
	logger->logAttribute("i_line22", line22->attribute("i_intf"));
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
	Real timeStep = 0.0001;
	Bool startFaultEvent=true;
	// Bool startFaultEvent= false;
	Bool endFaultEvent=true;
	Real startTimeFault=1;
	Real endTimeFault=1.5;

	//Transient Stability model(classical model 2nd order)
	SP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//Transient Stability model(classical model 2nd order)
	DP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

}
