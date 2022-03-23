#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
Examples::Grids::SMIB::ScenarioConfig3 GridParams;

// Generator parameters
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;


void EMT_3ph_SynGen_Load(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchOpen,
	Real switchClosed, int SGModel, Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);
	
	// Nodes
	std::vector<Complex> initialVoltage_n1{ GridParams.initTerminalVolt, 
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_B,
											GridParams.initTerminalVolt * SHIFT_TO_PHASE_C
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);

	// Components
	// Synchronous generator
	std::shared_ptr<EMT::Ph3::ReducedOrderSynchronGeneratorVBR> genEMT = nullptr;
	if (SGModel==3)
		genEMT = EMT::Ph3::SynchronGenerator3OrderVBR::make("SynGen", logLevel);
	else if (SGModel==4)
		genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make("SynGen", logLevel);
	else if (SGModel==6)
		genEMT = EMT::Ph3::SynchronGenerator6aOrderVBR::make("SynGen", logLevel);
	else if (SGModel==7)
		genEMT = EMT::Ph3::SynchronGenerator6bOrderVBR::make("SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    genEMT->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower, 
							 GridParams.initTerminalVolt);

	auto load = CPS::EMT::Ph3::RXLoad::make("Load", logLevel);
	load->setParameters(Math::singlePhaseParameterToThreePhase(GridParams.initActivePower/3), 
						Math::singlePhaseParameterToThreePhase(GridParams.initReactivePower/3),
						GridParams.VnomMV);

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", logLevel);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(switchOpen), 
						 Math::singlePhaseParameterToThreePhase(switchClosed));
	fault->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	load->connect({ n1EMT });
	fault->connect({EMT::SimNode::GND, n1EMT});

	SystemTopology systemEMT;
	if (SGModel==3)
		systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator3OrderVBR>(genEMT), load, fault});
	else if (SGModel==4)
		systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator4OrderVBR>(genEMT), load, fault});
	else if (SGModel==6)
		systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator6aOrderVBR>(genEMT), load, fault});
	else if (SGModel==7)
		systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator6bOrderVBR>(genEMT), load, fault});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->addAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->addAttribute("Etorque", 	genEMT->attribute("Etorque"));
    //loggerEMT->addAttribute("delta", 	genEMT->attribute("delta"));
    //loggerEMT->addAttribute("w_r", 		genEMT->attribute("w_r"));
	//loggerEMT->addAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	//loggerEMT->addAttribute("Idq0", 	genEMT->attribute("Idq0"));
	//loggerEMT->addAttribute("Edq0", 	genEMT->attribute("Edq0_t"));
	//loggerEMT->addAttribute("Evbr", 	genEMT->attribute("Evbr"));

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent3Ph::make(startTimeFault, fault, true);
	simEMT.addEvent(sw1);
	auto sw2 = SwitchEvent3Ph::make(endTimeFault, fault, false);
	simEMT.addEvent(sw2);
	
	simEMT.run();
	simEMT.logStepTimes(simNameEMT + "_step_times");
}

int main(int argc, char* argv[]) {	

	// Simultion parameters
	Real SwitchClosed = GridParams.SwitchClosed;
	Real SwitchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 5;
	Real timeStep = 1e-3;
	int SGModel = 4;
	Real H = syngenKundur.H;
	std::string SGModel_str = "4Order";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("SGModel") != args.options.end()) {
			SGModel = args.getOptionReal("SGModel");
			if (SGModel==3)
				SGModel_str = "3Order";
			else if (SGModel==4)
				SGModel_str = "4Order";
			else if (SGModel==6)
				/// 6th order model (Marconato's model)
				SGModel_str = "6aOrder";
			else if (SGModel==7)
				/// 6th order model (Andorson-Fouad's model)
				SGModel_str = "6bOrder";
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
	std::string simName ="EMT_SynGen" + SGModel_str + "VBR_Load_Fault" + stepSize_str + inertia_str;
	EMT_3ph_SynGen_Load(simName, timeStep, finalTime, H, startTimeFault, endTimeFault, 
						 logDownSampling, SwitchOpen, SwitchClosed, SGModel, logLevel);
}