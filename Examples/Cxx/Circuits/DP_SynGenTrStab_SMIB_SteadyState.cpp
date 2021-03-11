#include <DPsim.h>


using namespace DPsim;
using namespace CPS;


//-----------Power system-----------//
//Voltage level as Base Voltage
Real Vnom = 230e3;

//-----------Generator-----------//
Real nomPower = 500e6;
Real nomPhPhVoltRMS = 22e3;
Real nomFreq = 60;
Real nomOmega= nomFreq* 2*PI;
Real H = 5;
Real Xpd=0.31;
Real Rs = 0.003*0;
Real Kd = 1;
// Initialization parameters
Real initMechPower= 300e6;
Real initActivePower = 300e6;
Real setPointVoltage=nomPhPhVoltRMS + 0.05*nomPhPhVoltRMS;

//-----------Transformer-----------//
Real t_ratio=Vnom/nomPhPhVoltRMS;

//PiLine parameters calculated from CIGRE Benchmark system
Real lineResistance = 6.7;
Real lineInductance = 47./nomOmega;
Real lineCapacitance = 3.42e-4/nomOmega;
Real lineConductance =0;

// Parameters for powerflow initialization
// Slack voltage: 1pu
Real Vslack = Vnom;

void DP_1ph_SynGenTrStab_SteadyState(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia) {
	//  // ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	genPF->setParameters(nomPower, nomPhPhVoltRMS, initActivePower, setPointVoltage*t_ratio, PowerflowBusType::PV);
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
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

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

	// Components
	auto genDP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	genDP->setStandardParametersPU(nomPower, nomPhPhVoltRMS, nomFreq, Xpd*std::pow(t_ratio,2), cmdInertia*H, Rs, Kd );
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower= genPF->getApparentPower();
	genDP->setInitialValues(initApparentPower, initMechPower);

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetDP->setParameters(Vslack);
	// Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineDP->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);

	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{genDP, lineDP, extnetDP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);


	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));
	//gen
	loggerDP->addAttribute("Ep", genDP->attribute("Ep"));
	loggerDP->addAttribute("v_gen", genDP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen", genDP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen", genDP->attribute("w_r"));
	loggerDP->addAttribute("delta_r_gen", genDP->attribute("delta_r"));
	loggerDP->addAttribute("P_elec", genDP->attribute("P_elec"));
	loggerDP->addAttribute("P_mech", genDP->attribute("P_mech"));
	//line
	loggerDP->addAttribute("v_line", lineDP->attribute("v_intf"));
	loggerDP->addAttribute("i_line", lineDP->attribute("i_intf"));
	//slack
	loggerDP->addAttribute("v_slack", extnetDP->attribute("v_intf"));
	loggerDP->addAttribute("i_slack", extnetDP->attribute("i_intf"));



	Simulation simDP(simNameDP, Logger::Level::debug);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.addLogger(loggerDP);

	simDP.run();
}

int main(int argc, char* argv[]) {	
		

	//Simultion parameters
	String simName="DP_SynGenTrStab_SMIB_SteadyState";
	Real finalTime = 10;
	Real timeStep = 0.001;
	Bool startFaultEvent=false;
	Bool endFaultEvent=false;
	Real startTimeFault=10;
	Real endTimeFault=10.1;
	Real cmdInertia= 1.0;

	DP_1ph_SynGenTrStab_SteadyState(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia);
}