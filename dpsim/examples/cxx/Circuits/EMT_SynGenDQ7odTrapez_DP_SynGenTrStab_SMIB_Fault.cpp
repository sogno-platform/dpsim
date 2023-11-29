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
using namespace CPS::CIM;

// General grid parameters
Real VnomMV = 24e3;
Real VnomHV = 230e3;
Real nomFreq = 60;
Real ratio = VnomMV/VnomHV;
Real nomOmega= nomFreq* 2*PI;


Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Examples::Grids::CIGREHVAmerican::LineParameters lineCIGREHV;

// HV line parameters referred to MV side
Real lineLength = 100;
Real lineResistance = lineCIGREHV.lineResistancePerKm*lineLength*std::pow(ratio,2);
Real lineInductance = lineCIGREHV.lineReactancePerKm*lineLength*std::pow(ratio,2)/nomOmega;
Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm*lineLength/std::pow(ratio,2)/nomOmega;
Real lineConductance = 8e-2; //change to allow bigger time steps and to stabilize simulation (8e-2 used for 10us)


//Breaker to trigger fault between the two lines
Real BreakerOpen = 1e9;
Real BreakerClosed = 0.001;
// Real BreakerClosed = 0.1;

// Parameters for powerflow initialization
Real Vnom = VnomMV;
// Slack voltage
Real Vslack = Vnom;

//Synchronous generator
Real setPointActivePower=300e6;
Real setPointVoltage=Vnom+0.05*Vnom;

void EMT_3ph_SynGenDQ7odTrapez_ThreePhFault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault){

	// ----- POWERFLOW FOR INITIALIZATION -----

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = "EMT_PFinit";
	Logger::setLogDir("logs/" + simNamePF);
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, syngenKundur.nomVoltage, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(VnomMV);
	extnetPF->setBaseVoltage(VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(VnomMV);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(nomFreq,
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
	String simName = "EMT_3ph_SynGenDQ7odTrapez_ThreePhFault";
	Logger::setLogDir("logs/"+simName);

	// Extract relevant powerflow results
	Real initTerminalVolt=std::abs(n1PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle= Math::phase(n1PF->singleVoltage()); // angle in rad
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Real initMechPower = initActivePower;

	// Nodes
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);

	// Components
	//Synch
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, syngenKundur.poleNum, syngenKundur.nomFieldCurr,
		syngenKundur.Rs, syngenKundur.Ll, syngenKundur.Lmd, syngenKundur.Lmq, syngenKundur.Rfd, syngenKundur.Llfd, syngenKundur.Rkd, syngenKundur.Llkd, syngenKundur.Rkq1, syngenKundur.Llkq1, syngenKundur.Rkq2, syngenKundur.Llkq2, syngenKundur.H,
		initActivePower, initReactivePower, initTerminalVolt,
		initVoltAngle, initMechPower);

	//Grid bus as Slack
	auto extnet = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line
	auto line = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	line->setParameters(Math::singlePhaseParameterToThreePhase(lineResistance),
	                      Math::singlePhaseParameterToThreePhase(lineInductance),
					      Math::singlePhaseParameterToThreePhase(lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(lineConductance));

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(BreakerOpen),
						 Math::singlePhaseParameterToThreePhase(BreakerClosed));
	fault->openSwitch();

	// Topology
	gen->connect({ n1 });
	line->connect({ n1, n2 });
	extnet->connect({ n2 });
	fault->connect({EMT::SimNode::GND, n1});
	auto system = SystemTopology(nomFreq,
			SystemNodeList{n1, n2},
			SystemComponentList{gen, line, fault, extnet});

	// Initialization of dynamic topology
	system.initWithPowerflow(systemPF, CPS::Domain::EMT);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line", line->attribute("v_intf"));
	logger->logAttribute("i_line", line->attribute("i_intf"));
	logger->logAttribute("v_gen", gen->attribute("v_intf"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("delta_r", gen->attribute("delta_r"));

	// Simulation
	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent3Ph::make(startTimeFault, fault, true);

		sim.addEvent(sw1);
		}


	if(endFaultEvent){

		auto sw2 = SwitchEvent3Ph::make(endTimeFault, fault, false);
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
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, syngenKundur.nomVoltage, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(VnomMV);

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
	gen->setFundamentalParametersPU(syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, syngenKundur.Ll, syngenKundur.Lmd, syngenKundur.Llfd, syngenKundur.H, 1.5);

	//Grid bus as Slack
	auto extnet = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line
	auto line = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	line->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);

	// //Breaker
	// auto fault = CPS::DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	// fault->setParameters(BreakerOpen, BreakerClosed);
	// fault->open();

	//Switch
	auto fault = CPS::DP::Ph1::varResSwitch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->setInitParameters(timeStep);
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
	system.initWithPowerflow(systemPF, CPS::Domain::DP);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line", line->attribute("v_intf"));
	logger->logAttribute("i_line", line->attribute("i_intf"));
	logger->logAttribute("v_gen", gen->attribute("v_intf"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("delta_r", gen->attribute("delta_r"));
	logger->logAttribute("Ep", gen->attribute("Ep_mag"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	sim.addLogger(logger);
	sim.doSystemMatrixRecomputation(true);

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

void SP_1ph_SynGenTrStab_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault) {
	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "SP_PFinit";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, syngenKundur.nomVoltage, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(VnomMV);

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
	gen->setFundamentalParametersPU(syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, syngenKundur.Ll, syngenKundur.Lmd, syngenKundur.Llfd, syngenKundur.H, 1.5);

	//Grid bus as Slack
	auto extnet = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line
	auto line = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	line->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);

	// //Breaker
	// auto fault = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	// fault->setParameters(BreakerOpen, BreakerClosed);
	// fault->open();

	//Switch
	auto fault = CPS::SP::Ph1::varResSwitch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(BreakerOpen, BreakerClosed);
	fault->setInitParameters(timeStep);
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
	system.initWithPowerflow(systemPF, CPS::Domain::SP);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("v2", n2->attribute("v"));
	logger->logAttribute("v_line", line->attribute("v_intf"));
	logger->logAttribute("i_line", line->attribute("i_intf"));
	logger->logAttribute("v_gen", gen->attribute("v_intf"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("delta_r", gen->attribute("delta_r"));
	logger->logAttribute("Ep", gen->attribute("Ep_mag"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(logger);
	sim.doSystemMatrixRecomputation(true);

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
	Real finalTime = 3;
	Real timeStep = 0.00005;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=1;
	Real endTimeFault=1.1;

	//Full order DQ model with trapezoidal integration method (7th order)
	EMT_3ph_SynGenDQ7odTrapez_ThreePhFault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//Transient Stability model(classical model 2nd order)
	DP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);
	SP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);
}
