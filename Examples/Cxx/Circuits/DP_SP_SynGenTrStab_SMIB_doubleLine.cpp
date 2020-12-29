
#include <DPsim.h>


using namespace DPsim;
using namespace CPS;


//-----------Power system-----------//
//Voltage level as Base Voltage
Real Vnom = 230e3;
// Real Snom = 100e6;


//-----------Generator-----------//
Real nomPower = 500e6;
Real nomPhPhVoltRMS = 22e3;
Real nomFreq = 60;
Real H = 5;
Real Xpd=0.4+0.1; //in p.u
Real Rs = 0.003*0;
Real Kd = 1;
// Initialization parameters
Real initActivePower = 300e6;
Real initMechPower= 300e6;
Real setPointVoltage=nomPhPhVoltRMS+0.05*nomPhPhVoltRMS;
Real initReactivePower = 0;

// Define machine parameters in per unit
Real nomFieldCurr = 1300;
Int poleNum = 2;
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
Real fieldVoltage = 7.0821;

// Turbine governor
Real Ta = 0.3;
Real Fa = 0.3;
Real Tb = 7;
Real Fb = 0.3;
Real Tc = 0.2;
Real Fc = 0.4;
Real Tsr = 0.1;
Real Tsm = 0.3;
Real K = 20;

//PiLine parameters calculated from CIGRE Benchmark system
Real lineResistance = 6.7;
Real lineInductance = 47./377;
Real lineCapacitance = 3.42e-4/377;
//change inductance to allow bigger time steps and to stabilize simulation 8e-2(10us)
//Real lineConductance =9e-4;
Real lineConductance =1e-3;

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
//Real BreakerOpen = 5e2;
Real BreakerOpen = 2e2;
//Real BreakerClosed = 0.001;
Real BreakerClosed = 0.08;

// Parameters for powerflow initialization
// Slack voltage: 1pu
Real Vslack = Vnom;

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
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(nomPower, nomPhPhVoltRMS, initActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(nomPhPhVoltRMS);
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

	//Breaker
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(BreakerOpen, BreakerClosed);
	faultPF->open();


	// Topology
	genPF->connect({ n1PF });
	linePF1->connect({ n1PF, n3PF });
	linePF21->connect({ n1PF, n2PF });
	linePF22->connect({ n2PF, n3PF });
	extnetPF->connect({ n3PF });
	faultPF->connect({CPS::SP::SimNode::GND, n2PF});
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{genPF, linePF1, linePF21, faultPF, linePF22, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_line1", linePF1->attribute("v_intf"));
	loggerPF->addAttribute("i_line1", linePF1->attribute("i_intf"));
	loggerPF->addAttribute("v_line21", linePF21->attribute("v_intf"));
	loggerPF->addAttribute("i_line21", linePF21->attribute("i_intf"));
	loggerPF->addAttribute("v_line22", linePF22->attribute("v_intf"));
	loggerPF->addAttribute("i_line22", linePF22->attribute("i_intf"));
	loggerPF->addAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->addAttribute("ig", genPF->attribute("i_intf"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doPowerFlowInit(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simName = "SP_1ph_SynGenTrStab_Fault_dl";
	Logger::setLogDir("logs/"+simName);
	
	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
	auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);

	// Components
	auto gen = CPS::SP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	//gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);
	gen->setStandardParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Xpd, H, Rs, Kd );
	gen->setInitialValues(initActivePower, initMechPower);

	// Governor
	//gen->addGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm, initActivePower / nomPower, initMechPower / nomPower);


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
	
	// //add series capacitance & Resistance in parallel to switch
	// auto switchBranchRes= SP::Ph1::Resistor::make("switchBranchRes", Logger::Level::debug);
	// switchBranchRes->setParameters(1e3);
	// switchBranchRes->setBaseVoltage(Vnom);

	// auto switchBranchCap= SP::Ph1::Capacitor::make("switchBranchCap", Logger::Level::debug);
	// switchBranchCap->setParameters(1);
	// // switchBranchCap->setBaseVoltage(Vnom);

	// // Topology
	// gen->connect({ n1 });
	// line1->connect({ n1, n3 });
	// line21->connect({ n1, n2 });
	// line22->connect({ n2, n3 });
	// extnet->connect({ n3 });
	// fault->connect({CPS::SP::SimNode::GND, n2});
	// switchBranchRes->connect({n2, n4} );
	// switchBranchCap->connect({CPS::SP::SimNode::GND, n4});
	// auto system = SystemTopology(60,
	// 		SystemNodeList{n1, n2, n3, n4},
	// 		SystemComponentList{gen, line1, line21, fault, switchBranchRes, switchBranchCap, line22, extnet});

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
	CIM::Reader reader(simName, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, system);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v_line1", line1->attribute("v_intf"));
	logger->addAttribute("i_line1", line1->attribute("i_intf"));
	logger->addAttribute("v_line21", line21->attribute("v_intf"));
	logger->addAttribute("i_line21", line21->attribute("i_intf"));
	logger->addAttribute("v_line22", line22->attribute("v_intf"));
	logger->addAttribute("i_line22", line22->attribute("i_intf"));
	logger->addAttribute("v_gen", gen->attribute("v_intf"));
	logger->addAttribute("i_gen", gen->attribute("i_intf"));
	logger->addAttribute("wr_gen", gen->attribute("w_r"));
	logger->addAttribute("delta_r_gen", gen->attribute("delta_r"));
	logger->addAttribute("Ep", gen->attribute("Ep_mag"));

	logger->addAttribute("P_elec", gen->attribute("P_elec"));
	logger->addAttribute("P_mech", gen->attribute("P_mech"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.doPowerFlowInit(false);
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
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(nomPower, nomPhPhVoltRMS, initActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(nomPhPhVoltRMS);
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

	//Fault Breaker system
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(BreakerOpen, BreakerClosed);
	faultPF->open();


	// Topology
	genPF->connect({ n1PF });
	linePF1->connect({ n1PF, n3PF });
	linePF21->connect({ n1PF, n2PF });
	linePF22->connect({ n2PF, n3PF });
	faultPF->connect({CPS::SP::SimNode::GND, n2PF});
	extnetPF->connect({ n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{genPF, linePF1, linePF21, faultPF, linePF22, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_line1", linePF1->attribute("v_intf"));
	loggerPF->addAttribute("i_line1", linePF1->attribute("i_intf"));
	loggerPF->addAttribute("v_line21", linePF21->attribute("v_intf"));
	loggerPF->addAttribute("i_line21", linePF21->attribute("i_intf"));
	loggerPF->addAttribute("v_line22", linePF22->attribute("v_intf"));
	loggerPF->addAttribute("i_line22", linePF22->attribute("i_intf"));
	loggerPF->addAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->addAttribute("ig", genPF->attribute("i_intf"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doPowerFlowInit(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simName = "DP_1ph_SynGenTrStab_Fault_dl";
	Logger::setLogDir("logs/"+simName);
	
	// Nodes
	auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
	auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);

	// Components
	auto gen = CPS::DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	//gen->setFundamentalParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Ll, Lmd, Llfd, H);
	gen->setStandardParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Xpd, H, Rs, Kd );
	gen->setInitialValues(initActivePower, initMechPower);

	// // Governor
	// gen->addGovernor(Ta, Tb, Tc, Fa, Fb, Fc, K, Tsr, Tsm, initActivePower / nomPower, initMechPower / nomPower);

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

	//add series capacitance & Resistance in parallel to switch
	// auto switchBranchRes= SP::Ph1::Resistor::make("switchBranchRes", Logger::Level::debug);
	// switchBranchRes->setParameters(1e3);
	// switchBranchRes->setBaseVoltage(Vnom);

	// auto switchBranchCap= SP::Ph1::Capacitor::make("switchBranchCap", Logger::Level::debug);
	// switchBranchCap->setParameters(1);
	// //switchBranchCap->setBaseVoltage(Vnom);

	// // Topology
	// gen->connect({ n1 });
	// line1->connect({ n1, n3 });
	// line21->connect({ n1, n2 });
	// line22->connect({ n2, n3 });
	// extnet->connect({ n3 });
	// fault->connect({CPS::SP::SimNode::GND, n2});
	// switchBranchRes->connect({n2, n4} );
	// switchBranchCap->connect({CPS::SP::SimNode::GND, n4});
	// auto system = SystemTopology(60,
	// 		SystemNodeList{n1, n2, n3, n4},
	// 		SystemComponentList{gen, line1, line21, fault, switchBranchRes, switchBranchCap, line22, extnet});

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
	CIM::Reader reader(simName, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, system);


	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v3", n3->attribute("v"));
	logger->addAttribute("v_line1", line1->attribute("v_intf"));
	logger->addAttribute("i_line1", line1->attribute("i_intf"));
	logger->addAttribute("v_line21", line21->attribute("v_intf"));
	logger->addAttribute("i_line21", line21->attribute("i_intf"));
	logger->addAttribute("v_line22", line22->attribute("v_intf"));
	logger->addAttribute("i_line22", line22->attribute("i_intf"));
	logger->addAttribute("v_gen", gen->attribute("v_intf"));
	logger->addAttribute("i_gen", gen->attribute("i_intf"));
	logger->addAttribute("wr_gen", gen->attribute("w_r"));
	logger->addAttribute("delta_r_gen", gen->attribute("delta_r"));
	logger->addAttribute("Ep", gen->attribute("Ep_mag"));

	logger->addAttribute("P_elec", gen->attribute("P_elec"));
	logger->addAttribute("P_mech", gen->attribute("P_mech"));

	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	//sim.doPowerFlowInit(false);
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

void EMT_3ph_SynGenDQ7odTrapez_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault) {

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = "EMT_PFinit_dl";
	Logger::setLogDir("logs/" + simNamePF);
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(nomPower, nomPhPhVoltRMS, initActivePower, setPointVoltage, PowerflowBusType::PV);
	genPF->setBaseVoltage(nomPhPhVoltRMS);
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
		//Fault Breaker system
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(BreakerOpen, BreakerClosed);
	faultPF->open();


	// Topology
	genPF->connect({ n1PF });
	linePF1->connect({ n1PF, n3PF });
	linePF21->connect({ n1PF, n2PF });
	linePF22->connect({ n2PF, n3PF });
	faultPF->connect({CPS::SP::SimNode::GND, n2PF});
	extnetPF->connect({ n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{genPF, linePF1, linePF21, faultPF, linePF22, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_line21", linePF21->attribute("v_intf"));
	loggerPF->addAttribute("i_line21", linePF21->attribute("i_intf"));
	loggerPF->addAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->addAttribute("ig", genPF->attribute("i_intf"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doPowerFlowInit(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------	
	String simName = "EMT_3ph_SynGenDQ7odTrapez_Fault_dl";
	Logger::setLogDir("logs/"+simName);
	
	// angle in rad
	Real initTerminalVolt=std::abs(n1PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle= Math::phase(n1PF->singleVoltage());

	// Nodes	
	auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3 = SimNode<Real>::make("n3", PhaseType::ABC);

	// Components
	//Synch
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQTrapez::make("SynGen");
	gen->setParametersFundamentalPerUnit(
		nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
		Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
		initActivePower, initReactivePower, initTerminalVolt,
		initVoltAngle, fieldVoltage, initMechPower);
	
	//Grid bus as Slack
	auto extnet = EMT::Ph3::NetworkInjection::make("Slack", Logger::Level::debug);

	// Line1
	auto line1 = EMT::Ph3::PiLine::make("PiLine1", Logger::Level::debug);
	line1->setParameters(Math::singlePhaseParameterToThreePhase(lineResistance), 
	                      Math::singlePhaseParameterToThreePhase(lineInductance), 
					      Math::singlePhaseParameterToThreePhase(lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(lineConductance));
	// Line21
	auto line21 = EMT::Ph3::PiLine::make("PiLine21", Logger::Level::debug);
	line21->setParameters(Math::singlePhaseParameterToThreePhase(lineResistance21), 
	                      Math::singlePhaseParameterToThreePhase(lineInductance21), 
					      Math::singlePhaseParameterToThreePhase(lineCapacitance21),
						  Math::singlePhaseParameterToThreePhase(lineConductance21));
	// Line22
	auto line22 = EMT::Ph3::PiLine::make("PiLine22", Logger::Level::debug);
	line22->setParameters(Math::singlePhaseParameterToThreePhase(lineResistance22), 
	                      Math::singlePhaseParameterToThreePhase(lineInductance22), 
					      Math::singlePhaseParameterToThreePhase(lineCapacitance22),
						  Math::singlePhaseParameterToThreePhase(lineConductance22));
				  
	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", Logger::Level::debug);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(BreakerOpen), 
						 Math::singlePhaseParameterToThreePhase(BreakerClosed));
	fault->openSwitch();

	// Topology
	gen->connect({ n1 });
	line1->connect({ n1, n3 });
	line21->connect({ n1, n2 });
	line22->connect({ n2, n3 });
	extnet->connect({ n3 });
	fault->connect({EMT::SimNode::GND, n2});
	auto system = SystemTopology(60,
			SystemNodeList{n1, n2, n3},
			SystemComponentList{gen, line1, line21, line22, fault, extnet});

	// Initialization of dynamic topology
	CIM::Reader reader(simName, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, system);

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("v2", n2->attribute("v"));
	logger->addAttribute("v_line21", line21->attribute("v_intf"));
	logger->addAttribute("i_line21", line21->attribute("i_intf"));
	logger->addAttribute("v_gen", gen->attribute("v_intf"));
	logger->addAttribute("i_gen", gen->attribute("i_intf"));
	logger->addAttribute("wr_gen", gen->attribute("w_r"));
	logger->addAttribute("delta_r_gen", gen->attribute("delta_r"));

	// Simulation
	Simulation sim(simName, Logger::Level::debug);
	sim.setSystem(system);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.doPowerFlowInit(false);
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


int main(int argc, char* argv[]) {		

	//Simultion parameters
	Real finalTime = 50;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	// Bool startFaultEvent= false;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.2;
	//Real endTimeFault=0.3401;

	//Transient Stability model(classical model 2nd order)
	SP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//Transient Stability model(classical model 2nd order)
	DP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);
	
	//EMT reference 50us
	//EMT_3ph_SynGenDQ7odTrapez_Fault(1e-6, 2 ,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//loop for conductance value

	//loop for time step

	//loop for CCT

}