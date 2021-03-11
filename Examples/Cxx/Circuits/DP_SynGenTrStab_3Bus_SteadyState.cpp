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
//Real initReactivePower_G1 = 106e6;
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

void DP_SynGenTrStab_3Bus_SteadyState(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia) {
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

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	loadPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, loadPF, line12PF, line13PF, line23PF});

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
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/"+simNameDP);
	
	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3DP = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	//Synchronous generator 1
	auto gen1DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1DP->setStandardParametersPU(nomPower_G1, nomPhPhVoltRMS_G1, nomFreq_G1, Xpd_G1*std::pow(t1_ratio,2), cmdInertia*H_G1, Rs_G1, Kd_G1 );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= gen1PF->getApparentPower();
	gen1DP->setInitialValues(initApparentPower_G1, initMechPower_G1);

	//Synchronous generator 2
	auto gen2DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2DP->setStandardParametersPU(nomPower_G2, nomPhPhVoltRMS_G2, nomFreq_G2, Xpd_G2*std::pow(t2_ratio,2), cmdInertia*H_G2, Rs_G2, Kd_G2 );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2= gen2PF->getApparentPower();
	gen2DP->setInitialValues(initApparentPower_G2, initMechPower_G2);

	///Load
	auto loadDP=DP::Ph1::RXLoad::make("Load", Logger::Level::debug);
	loadDP->setParameters(activePower_L, reactivePower_L, Vnom);
	//Line12
	auto line12DP = DP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12DP->setParameters(lineResistance12, lineInductance12, lineCapacitance12, lineConductance12);
	//Line13
	auto line13DP = DP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13DP->setParameters(lineResistance13, lineInductance13, lineCapacitance13, lineConductance13);
	//Line23
	auto line23DP = DP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23DP->setParameters(lineResistance23, lineInductance23, lineCapacitance23, lineConductance23);

		// Topology
	gen1DP->connect({ n1DP });
	gen2DP->connect({ n2DP });
	loadDP->connect({ n3DP });
	line12DP->connect({ n1DP, n2DP });
	line13DP->connect({ n1DP, n3DP });
	line23DP->connect({ n2DP, n3DP });
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP, n3DP},
			SystemComponentList{gen1DP, gen2DP, loadDP, line12DP, line13DP, line23DP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));
	loggerDP->addAttribute("v3", n3DP->attribute("v"));
	loggerDP->addAttribute("v_line12", line12DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line12", line12DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line13", line13DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line13", line13DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line23", line23DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line23", line23DP->attribute("i_intf"));
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
	sim.addLogger(loggerDP);

	sim.run();
}

int main(int argc, char* argv[]) {	
		

	//Simultion parameters
	String simName="DP_SynGenTrStab_3Bus_SteadyState";
	Real finalTime = 10;
	Real timeStep = 0.001;
	Bool startFaultEvent=false;
	Bool endFaultEvent=false;
	Real startTimeFault=10;
	Real endTimeFault=10.1;
	Real cmdInertia= 1.0;

	DP_SynGenTrStab_3Bus_SteadyState(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia);
}