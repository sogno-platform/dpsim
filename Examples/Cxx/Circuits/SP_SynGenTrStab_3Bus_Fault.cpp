#include <DPsim.h>


using namespace DPsim;
using namespace CPS;


//-----------Power system-----------//
//Voltage level as Base Voltage
Real Vnom = 230e3;

//-----------Generator 1 (bus1)-----------//
//Machine parameters
Real nomPower_G1 = 300e6;
Real nomPhPhVoltRMS_G1 = 25e3;
Real nomFreq_G1 = 60;
Real H_G1 = 6;
Real Xpd_G1=0.3; //in p.u
Real Rs_G1 = 0.003*0;
Real Kd_G1 = 1;
// Initialization parameters 
Real initActivePower_G1 = 270e6;
Real initMechPower_G1 = 270e6;
Real setPointVoltage_G1=nomPhPhVoltRMS_G1+0.05*nomPhPhVoltRMS_G1;

//-----------Generator 2 (bus2)-----------//
//Machine parameters in per unit
Real nomPower_G2 = 50e6;
Real nomPhPhVoltRMS_G2 = 13.8e3;
Real nomFreq_G2 = 60;
Real H_G2 = 2;
Real Xpd_G2=0.1; //in p.u
Real Rs_G2 = 0.003*0;
Real Kd_G2 =1;
// Initialization parameters 
Real initActivePower_G2 = 45e6;
// Real initReactivePower_G2 = 106e6;
Real initMechPower_G2 = 45e6;
Real setPointVoltage_G2=nomPhPhVoltRMS_G2-0.05*nomPhPhVoltRMS_G2;

//-----------Transformers-----------//
Real t1_ratio=Vnom/nomPhPhVoltRMS_G1;
Real t2_ratio=Vnom/nomPhPhVoltRMS_G2;

//-----------Load (bus3)-----------
Real activePower_L= 310e6;
Real reactivePower_L= 150e6;

//-----------Transmission Lines-----------//
//PiLine parameters
//line 1-2 (180km)
Real lineResistance12 = 0.04*180;
Real lineInductance12 = (0.4/377)*180;
Real lineCapacitance12 = (4.3e-6/377)*180;
// Real lineCapacitance12 =0;
Real lineConductance12 =0;
//line 1-3 (150km)
Real lineResistance13 = 0.0267*150;
Real lineInductance13 = (0.267/377)*150;
Real lineCapacitance13 = (4.3e-6/377)*150;
Real lineConductance13 =0;
//line 2-3 (80km)
Real lineResistance23 = 0.04*80;
Real lineInductance23 = (0.267/377)*80;
Real lineCapacitance23 = (4.3e-6/377)*80;
Real lineConductance23 =0;

//Switch to trigger fault at generator terminal
Real SwitchOpen = 1e12;
Real SwitchClosed = 0.1;

void SP_SynGenTrStab_3Bus_Fault(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia) {
	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(nomPower_G1, nomPhPhVoltRMS_G1, initActivePower_G1, setPointVoltage_G1*t1_ratio, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(nomPower_G2, nomPhPhVoltRMS_G2, initActivePower_G2, setPointVoltage_G2*t2_ratio, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(Vnom);

	//use Shunt as Load for powerflow
	auto loadPF = SP::Ph1::Shunt::make("Load", Logger::Level::debug);
	loadPF->setParameters(activePower_L / std::pow(Vnom, 2), - reactivePower_L / std::pow(Vnom, 2));
	loadPF->setBaseVoltage(Vnom);
	
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
	//Switch
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(SwitchOpen, SwitchClosed);
	faultPF->open();

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	loadPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	faultPF->connect({SP::SimNode::GND , n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, loadPF, line12PF, line13PF, line23PF, faultPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameSP = simName + "_SP";
	Logger::setLogDir("logs/"+simNameSP);
	
	// Nodes
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3SP = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	//Synchronous generator 1
	auto gen1SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1SP->setStandardParametersPU(nomPower_G1, nomPhPhVoltRMS_G1, nomFreq_G1, Xpd_G1*std::pow(t1_ratio,2), cmdInertia*H_G1, Rs_G1, Kd_G1 );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= gen1PF->getApparentPower();
	gen1SP->setInitialValues(initApparentPower_G1, initMechPower_G1);

	//Synchronous generator 2
	auto gen2SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2SP->setStandardParametersPU(nomPower_G2, nomPhPhVoltRMS_G2, nomFreq_G2, Xpd_G2*std::pow(t2_ratio,2), cmdInertia*H_G2, Rs_G2, Kd_G2 );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2= gen2PF->getApparentPower();
	gen2SP->setInitialValues(initApparentPower_G2, initMechPower_G2);

	//Load
	auto loadSP = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadSP->setParameters(activePower_L, reactivePower_L, Vnom);
	
	//Line12
	auto line12SP = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12SP->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	//Line13
	auto line13SP = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13SP->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	//Line23
	auto line23SP = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23SP->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);
	//Switch
	auto faultSP = SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultSP->setParameters(SwitchOpen, SwitchClosed);
	faultSP->open();

		// Topology
	gen1SP->connect({ n1SP });
	gen2SP->connect({ n2SP });
	loadSP->connect({ n3SP });
	line12SP->connect({ n1SP, n2SP });
	line13SP->connect({ n1SP, n3SP });
	line23SP->connect({ n2SP, n3SP });
	faultSP->connect({SP::SimNode::GND , n2SP });
	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP, n2SP, n3SP},
			SystemComponentList{gen1SP, gen2SP, loadSP, line12SP, line13SP, line23SP, faultSP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameSP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemSP);

	// Logging
	auto loggerSP = DataLogger::make(simNameSP);
	loggerSP->addAttribute("v1", n1SP->attribute("v"));
	loggerSP->addAttribute("v2", n2SP->attribute("v"));
	loggerSP->addAttribute("v3", n3SP->attribute("v"));
	loggerSP->addAttribute("v_line12", line12SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line12", line12SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line13", line13SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line13", line13SP->attribute("i_intf"));
	loggerSP->addAttribute("v_line23", line23SP->attribute("v_intf"));
	loggerSP->addAttribute("i_line23", line23SP->attribute("i_intf"));
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
	////ADD LOAD v_intf & i_intf to log attributes
	// loggerSP->addAttribute("v_load", loadSP->attribute("v_intf"));
	// loggerSP->addAttribute("i_load", loadSP->attribute("i_intf"));
	//Switch
	loggerSP->addAttribute("i_fault", faultSP->attribute("i_intf"));

	loggerSP->addAttribute("P_elec2", gen2SP->attribute("P_elec"));

	Simulation simSP(simNameSP, Logger::Level::debug);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.addLogger(loggerSP);

		// Events
	if (startFaultEvent){
		auto sw1 = SwitchEvent::make(startTimeFault, faultSP, true);

		simSP.addEvent(sw1);
	}

	if(endFaultEvent){

		auto sw2 = SwitchEvent::make(endTimeFault, faultSP, false);
		simSP.addEvent(sw2);
	
	}

	simSP.run();
}

int main(int argc, char* argv[]) {	
		

	//Simultion parameters
	String simName="SP_SynGenTrStab_3Bus_Fault";
	Real finalTime = 20;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.1;
	Real cmdInertia= 1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA") != args.options.end())
			cmdInertia = args.options["SCALEINERTIA"];		
	}

	SP_SynGenTrStab_3Bus_Fault(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia);
}