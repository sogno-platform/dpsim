
#include <DPsim.h>


using namespace DPsim;
using namespace CPS;

//-----------Power system-----------//
//Voltage level as Base Voltage
Real Vnom = 250e3;

//-----------Generator 1 (bus1)-----------//
//Machine parameters
Real nomPower_G1 = 300e6;
Real nomPhPhVoltRMS_G1 = 25e3;
Real nomFreq_G1 = 60;
Real H_G1 = 6;
Real Xpd_G1=0.3+0.15; //in p.u
Real Rs_G1 = 0.003;
Real Kd_G1 = 1;
// Initialization parameters 
Real initActivePower_G1 = 270e6;
//Real initReactivePower_G1 = 106e6;
Real initMechPower_G1 = 270e6;
Real setPointVoltage_G1=nomPhPhVoltRMS_G1+0.05*nomPhPhVoltRMS_G1;

//-----------Generator 2 (bus2)-----------//
//Machine parameters in per unit
Real nomPower_G2 = 50e6;
Real nomPhPhVoltRMS_G2 = 13.8e3;
Real nomFreq_G2 = 60;
Real H_G2 = 2;
Real Xpd_G2=0.3+0.2; //in p.u
Real Rs_G2 = 0.003;
Real Kd_G2 =1;
// Initialization parameters 
Real initActivePower_G2 = 45e6;
// Real initReactivePower_G2 = 106e6;
Real initMechPower_G2 = 45e6;
Real setPointVoltage_G2=nomPhPhVoltRMS_G2-0.05*nomPhPhVoltRMS_G2;

//-----------Slack (bus3)-----------
// Parameters for powerflow initialization
// Slack voltage: 1pu
Real Vslack = Vnom;

//-----------Transmission Lines-----------//
//PiLine parameters based on V_nom
//line 1-2 (180km)
Real lineResistance12 = 0.04*180;
Real lineInductance12 = (0.4/377)*180;
Real lineCapacitance12 = (4.3e-6/377)*180;
Real lineConductance12 =2e-4;
//line 1-3 (150km)
Real lineResistance13 = 0.0267*150;
Real lineInductance13 = (0.267/377)*150;
Real lineCapacitance13 = (4.3e-6/377)*150;
Real lineConductance13 =1.7e-4;
//line 2-3 (80km)
Real lineResistance23 = 0.04*80;
Real lineInductance23 = (0.267/377)*80;
Real lineCapacitance23 = (4.3e-6/377)*80;
Real lineConductance23 =9e-5;

//-----------Fault Breaker system-----------//
Real BreakerOpen = 1e2;
Real BreakerClosed = 0.001;

void SP_1ph_SynGenTrStab_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault){

	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "SP_PFinit_3BusSystem_infBus";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1 ideal model
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator1", Logger::Level::debug);
	gen1PF->setParameters(nomPower_G1, nomPhPhVoltRMS_G1, initActivePower_G1, setPointVoltage_G1, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(nomPhPhVoltRMS_G1);
	gen1PF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Synchronous generator 2 ideal model
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator2", Logger::Level::debug);
	gen2PF->setParameters(nomPower_G2, nomPhPhVoltRMS_G2, initActivePower_G2, setPointVoltage_G2, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(nomPhPhVoltRMS_G2);
	gen2PF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Load
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line12
	auto line12PF = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12PF->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	line12PF->setBaseVoltage(Vnom);
	//Line13
	auto line13PF = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13PF->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	line13PF->setBaseVoltage(Vnom);
	//Line23
	auto line23PF = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23PF->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);
	line23PF->setBaseVoltage(Vnom);


	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	extnetPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, extnetPF, line12PF, line13PF, line23PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));
	// loggerPF->addAttribute("v_line12", line12PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line12", line12PF->attribute("i_intf"));
	// loggerPF->addAttribute("v_line13", line13PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line13", line13PF->attribute("i_intf"));
	// loggerPF->addAttribute("v_line23", line23PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line23", line23PF->attribute("i_intf"));
	loggerPF->addAttribute("v_gen1", gen1PF->attribute("v_intf"));
	loggerPF->addAttribute("i_gen1", gen1PF->attribute("i_intf"));
	loggerPF->addAttribute("v_gen2", gen2PF->attribute("v_intf"));
	loggerPF->addAttribute("i_gen2", gen2PF->attribute("i_intf"));

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
	String simNameSP = "SP_1ph_3BusSystem_infBus";
	Logger::setLogDir("logs/"+simNameSP);
	
	// Nodes
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3SP = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1 2nd order model
	auto gen1SP = CPS::SP::Ph1::SynchronGeneratorTrStab::make("Generator1", Logger::Level::debug);
	// Input of Xpd is in per unit but value is converted to absolute values.
	gen1SP->setStandardParametersPU(nomPower_G1, nomPhPhVoltRMS_G1, nomFreq_G1, Xpd_G1, Rs_G1, H_G1, Kd_G1 );
	gen1SP->setInitialValues(initActivePower_G1, initMechPower_G1);
	//Synchronous generator 2 2nd order model
	auto gen2SP = CPS::SP::Ph1::SynchronGeneratorTrStab::make("Generator2", Logger::Level::debug);
	// Input of Xpd is in per unit but value is converted to absolute values.
	gen2SP->setStandardParametersPU(nomPower_G2, nomPhPhVoltRMS_G2, nomFreq_G2, Xpd_G2, Rs_G2, H_G2, Kd_G2);
	gen2SP->setInitialValues(initActivePower_G2, initMechPower_G2);
	//Slack
	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetSP->setBaseVoltage(Vnom);
	//Line12
	auto line12SP = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12SP->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	//Line13
	auto line13SP = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13SP->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	//Line23
	auto line23SP = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23SP->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);
	//Fault Breaker
	auto faultSP = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultSP->setParameters(BreakerOpen, BreakerClosed);
	faultSP->open();
	
	// Topology
	gen1SP->connect({ n1SP });
	gen2SP->connect({ n2SP });
	faultSP->connect({ CPS::SP::SimNode::GND, n2SP});
	extnetSP->connect({ n3SP });
	line12SP->connect({ n1SP, n2SP });
	line13SP->connect({ n1SP, n3SP });
	line23SP->connect({ n2SP, n3SP });
	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP, n2SP, n3SP},
			SystemComponentList{gen1SP, gen2SP, extnetSP, faultSP, line12SP, line13SP, line23SP});



	// Initialization of dynamic topology
	CIM::Reader reader(simNameSP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemSP);


	// Logging
	auto loggerSP = DataLogger::make(simNameSP);
	loggerSP->addAttribute("v1", n1SP->attribute("v"));
	loggerSP->addAttribute("v2", n2SP->attribute("v"));
	loggerSP->addAttribute("v3", n3SP->attribute("v"));
	// loggerSP->addAttribute("v_line12", line12SP->attribute("v_intf"));
	// loggerSP->addAttribute("i_line12", line12SP->attribute("i_intf"));
	// loggerSP->addAttribute("v_line13", line13SP->attribute("v_intf"));
	// loggerSP->addAttribute("i_line13", line13SP->attribute("i_intf"));
	// loggerSP->addAttribute("v_line23", line23SP->attribute("v_intf"));
	// loggerSP->addAttribute("i_line23", line23SP->attribute("i_intf"));
	loggerSP->addAttribute("v_gen1", gen1SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen1", gen1SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen1", gen1SP->attribute("w_r"));
	loggerSP->addAttribute("delta_gen1", gen1SP->attribute("delta_r"));
	loggerSP->addAttribute("Ep_gen1", gen1SP->attribute("Ep_mag"));
	loggerSP->addAttribute("v_gen2", gen2SP->attribute("v_intf"));
	loggerSP->addAttribute("i_gen2", gen2SP->attribute("i_intf"));
	loggerSP->addAttribute("wr_gen2", gen2SP->attribute("w_r"));
	loggerSP->addAttribute("delta_gen2", gen2SP->attribute("delta_r"));
	loggerSP->addAttribute("Ep_gen2", gen2SP->attribute("Ep_mag"));

	loggerSP->addAttribute("P_elec1", gen1SP->attribute("P_elec"));
	loggerSP->addAttribute("P_elec2", gen2SP->attribute("P_elec"));

	Simulation sim(simNameSP, Logger::Level::debug);
	sim.setSystem(systemSP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.doPowerFlowInit(false);
	sim.addLogger(loggerSP);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent::make(startTimeFault, faultSP, true);

		sim.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent::make(endTimeFault, faultSP, false);
		sim.addEvent(sw2);
	
	}

	sim.run();
}

void DP_1ph_SynGenTrStab_Fault(Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault) {
	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "DP_PFinit_3BusSystem_infBus";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1 ideal model
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator1", Logger::Level::debug);
	gen1PF->setParameters(nomPower_G1, nomPhPhVoltRMS_G1, initActivePower_G1, setPointVoltage_G1, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(nomPhPhVoltRMS_G1);
	gen1PF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Synchronous generator 2 ideal model
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator2", Logger::Level::debug);
	gen2PF->setParameters(nomPower_G2, nomPhPhVoltRMS_G2, initActivePower_G2, setPointVoltage_G2, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(nomPhPhVoltRMS_G2);
	gen2PF->modifyPowerFlowBusType(PowerflowBusType::PV);
	//Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(Vslack);
	extnetPF->setBaseVoltage(Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	//Line12
	auto line12PF = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12PF->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	line12PF->setBaseVoltage(Vnom);
	//Line13
	auto line13PF = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13PF->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	line13PF->setBaseVoltage(Vnom);
	//Line23
	auto line23PF = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23PF->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);
	line23PF->setBaseVoltage(Vnom);

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	extnetPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, extnetPF, line12PF, line13PF, line23PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));
	loggerPF->addAttribute("v3", n3PF->attribute("v"));
	// loggerPF->addAttribute("v_line12", line12PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line12", line12PF->attribute("i_intf"));
	// loggerPF->addAttribute("v_line13", line13PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line13", line13PF->attribute("i_intf"));
	// loggerPF->addAttribute("v_line23", line23PF->attribute("v_intf"));
	// loggerPF->addAttribute("i_line23", line23PF->attribute("i_intf"));
	loggerPF->addAttribute("v_gen1", gen1PF->attribute("v_intf"));
	loggerPF->addAttribute("i_gen1", gen1PF->attribute("i_intf"));
	loggerPF->addAttribute("v_gen2", gen2PF->attribute("v_intf"));
	loggerPF->addAttribute("i_gen2", gen2PF->attribute("i_intf"));

	

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
	String simNameDP = "DP_1ph_3BusSystem_infBus";
	Logger::setLogDir("logs/"+simNameDP);
	
	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3DP = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	//Synchronous generator 1 2nd order model
	auto gen1DP = CPS::DP::Ph1::SynchronGeneratorTrStab::make("Generator1", Logger::Level::debug);
	// Input of Xpd is in per unit but value is converted to absolute values.
	gen1DP->setStandardParametersPU(nomPower_G1, nomPhPhVoltRMS_G1, nomFreq_G1, Xpd_G1, Rs_G1, H_G1, Kd_G1);
	gen1DP->setInitialValues(initActivePower_G1, initMechPower_G1);
	//Synchronous generator 2 2nd order model
	auto gen2DP = CPS::DP::Ph1::SynchronGeneratorTrStab::make("Generator2", Logger::Level::debug);
	// Input of Xpd is in per unit but value is converted to absolute values.
	gen2DP->setStandardParametersPU(nomPower_G2, nomPhPhVoltRMS_G2, nomFreq_G2, Xpd_G2, Rs_G2, H_G2, Kd_G2);
	gen2DP->setInitialValues(initActivePower_G2, initMechPower_G2);
	//Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	//Line12
	auto line12DP = DP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12DP->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	//Line13
	auto line13DP = DP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13DP->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	//Line23
	auto line23DP = DP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23DP->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);

	//Fault Breaker
	auto faultDP = CPS::DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultDP->setParameters(BreakerOpen, BreakerClosed);
	faultDP->open();
	
	// Topology
	gen1DP->connect({ n1DP });
	gen2DP->connect({ n2DP });
	extnetDP->connect({ n3DP });
	faultDP->connect({ CPS::SP::SimNode::GND, n2DP});
	line12DP->connect({ n1DP, n2DP });
	line13DP->connect({ n1DP, n3DP });
	line23DP->connect({ n2DP, n3DP });
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP, n3DP},
			SystemComponentList{gen1DP, gen2DP, extnetDP, faultDP, line12DP, line13DP, line23DP});



	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);


	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));
	loggerDP->addAttribute("v3", n3DP->attribute("v"));
	// loggerDP->addAttribute("v_line12", line12DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line12", line12DP->attribute("i_intf"));
	// loggerDP->addAttribute("v_line13", line13DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line13", line13DP->attribute("i_intf"));
	// loggerDP->addAttribute("v_line23", line23DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line23", line23DP->attribute("i_intf"));
	loggerDP->addAttribute("v_gen1", gen1DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen1", gen1DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen1", gen1DP->attribute("w_r"));
	loggerDP->addAttribute("delta_gen1", gen1DP->attribute("delta_r"));
	loggerDP->addAttribute("Ep_gen1", gen1DP->attribute("Ep_mag"));
	loggerDP->addAttribute("v_gen2", gen2DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen2", gen2DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen2", gen2DP->attribute("w_r"));
	loggerDP->addAttribute("delta_gen2", gen2DP->attribute("delta_r"));
	loggerDP->addAttribute("Ep_gen2", gen2DP->attribute("Ep_mag"));
	////ADD LOAD v_intf & i_intf to log attributes
	// loggerDP->addAttribute("v_load", loadDP->attribute("v_intf"));
	// loggerDP->addAttribute("i_load", loadDP->attribute("i_intf"));

	loggerDP->addAttribute("P_elec2", gen2DP->attribute("P_elec"));

	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::DP);
	//sim.doPowerFlowInit(false);
	sim.addLogger(loggerDP);

	// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent::make(startTimeFault, faultDP, true);

		sim.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent::make(endTimeFault, faultDP, false);
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
	Real startTimeFault=20;
	Real endTimeFault=20.2;
	//Real endTimeFault=0.3401;

	//Transient Stability model(classical model 2nd order)
	SP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);

	//Transient Stability model(classical model 2nd order)
	DP_1ph_SynGenTrStab_Fault(timeStep, finalTime,startFaultEvent, endFaultEvent, startTimeFault, endTimeFault);
	
}