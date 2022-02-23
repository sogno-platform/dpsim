#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
Examples::Grids::SMIB::ScenarioConfig2 GridParams;

// Generator parameters
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

void SP_1ph_SynGen_Fault(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchOpen,
	Real switchClosed, Logger::Level logLevel) {

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, GridParams.VnomMV, GridParams.setPointActivePower, 
						 GridParams.setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(GridParams.VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(GridParams.VnomMV);
	extnetPF->setBaseVoltage(GridParams.VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(GridParams.lineResistance, GridParams.lineInductance, 
						  GridParams.lineCapacitance, GridParams.lineConductance);
	linePF->setBaseVoltage(GridParams.VnomMV);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(0.1);
	simPF.setFinalTime(0.1);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- Dynamic simulation ------
	String simNameSP = simName;
	Logger::setLogDir("logs/" + simNameSP);
	
	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0), 0.0, 0.0};
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0), 0.0, 0.0};
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initialVoltage_n1);
	auto n2SP = SimNode<Complex>::make("n2SP", PhaseType::Single, initialVoltage_n2);

	// Components
	auto genSP = SP::Ph1::SynchronGenerator4OrderDCIM::make("SynGen", Logger::Level::debug);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, syngenKundur.H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    genSP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));

	//Grid bus as Slack
	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetSP->setParameters(GridParams.VnomMV);

    // Line
	auto lineSP = SP::Ph1::PiLine::make("PiLine", logLevel);
	lineSP->setParameters(GridParams.lineResistance, GridParams.lineInductance, 
						  GridParams.lineCapacitance, GridParams.lineConductance);
	
	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", logLevel);
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genSP->connect({ n1SP });
	lineSP->connect({ n1SP, n2SP });
	extnetSP->connect({ n2SP });
	fault->connect({SP::SimNode::GND, n1SP});
	auto systemSP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1SP, n2SP},
			SystemComponentList{genSP, lineSP, extnetSP, fault});

	// Logging
	auto loggerSP = DataLogger::make(simNameSP, true, logDownSampling);
	//loggerSP->addAttribute("v_slack", 	 extnetSP->attribute("v_intf"));
	//loggerSP->addAttribute("i_slack", 	 extnetSP->attribute("i_intf"));
	loggerSP->addAttribute("v_gen", 	 genSP->attribute("v_intf"));
    loggerSP->addAttribute("i_gen", 	 genSP->attribute("i_intf"));
    loggerSP->addAttribute("Etorque", 	 genSP->attribute("Etorque"));
    loggerSP->addAttribute("delta", 	 genSP->attribute("delta"));
    loggerSP->addAttribute("w_r", 		 genSP->attribute("w_r"));
	loggerSP->addAttribute("Edq0", 		 genSP->attribute("Edq_t"));
	loggerSP->addAttribute("Vdq0", 		 genSP->attribute("Vdq"));
	loggerSP->addAttribute("Idq0", 		 genSP->attribute("Idq"));

	Simulation simSP(simNameSP, logLevel);
	simSP.doInitFromNodesAndTerminals(true);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simSP.addLogger(loggerSP);
	//simSP.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
	simSP.addEvent(sw1);

	auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
	simSP.addEvent(sw2);
	
	simSP.run();
}

int main(int argc, char* argv[]) {	

	// Simulation parameters
	Real SwitchClosed = GridParams.SwitchClosed;
	Real SwitchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real timeStep = 1e-6;
	Real H = syngenKundur.H;
	Real finalTime = 20;

	// Command line args processing
	CommandLineArgs args(argc, argv);
	std::string stepSize_str = "";
	std::string inertia_str = "";
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("Inertia") != args.options.end())  {
			H = args.getOptionReal("Inertia");
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "SP_SynGen4OrderDCIM_SMIB_Fault" + stepSize_str + inertia_str;
	SP_1ph_SynGen_Fault(simName, timeStep, finalTime, H, startTimeFault, endTimeFault, 
			logDownSampling, SwitchOpen, SwitchClosed, logLevel);
}