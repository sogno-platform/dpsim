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
Complex initElecPower =Complex(300e6,0);
Real initMechPower= 300e6;
Real initActivePower = 300e6;
Real setPointVoltage=Vnom;
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

//PiLine parameters calculated from CIGRE Benchmark system
Real lineResistance = 6.7;
Real lineInductance = 47./nomOmega;
Real lineCapacitance = 3.42e-4/nomOmega;
Real lineConductance =1e-3;

// Parameters for powerflow initialization
// Slack voltage: 1pu
Real Vslack = Vnom;

void DP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia) {
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
	genPF->setParameters(nomPower, nomPhPhVoltRMS, initActivePower, setPointVoltage, PowerflowBusType::PV);
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

	loggerPF->addAttribute("v_gen", genPF->attribute("v_intf"));
	loggerPF->addAttribute("ig", genPF->attribute("i_intf"));
	loggerPF->addAttribute("v_line", linePF->attribute("v_intf"));
	loggerPF->addAttribute("i_line", linePF->attribute("i_intf"));
	loggerPF->addAttribute("v_slack", extnetPF->attribute("v_intf"));
	loggerPF->addAttribute("i_slack", extnetPF->attribute("i_intf"));



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
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/"+simNameDP);
	
	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	// Components
	auto genDP = CPS::DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	genDP->setStandardParametersPU(nomPower, Vnom, nomFreq, Xpd, cmdInertia*H, Rs, Kd );
	genDP->setInitialValues(initElecPower, initMechPower);

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
	simDP.doPowerFlowInit(false);
	simDP.addLogger(loggerDP);

	simDP.run();
}

int main(int argc, char* argv[]) {	
		

	//Simultion parameters
	String simName="DP_SynGenTrStab_SMIB";
	Real finalTime = 10;
	Real timeStep = 0.001;
	Bool startFaultEvent=false;
	Bool endFaultEvent=false;
	Real startTimeFault=10;
	Real endTimeFault=10.1;
	Real cmdInertia= 1.0;

	DP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia);
}