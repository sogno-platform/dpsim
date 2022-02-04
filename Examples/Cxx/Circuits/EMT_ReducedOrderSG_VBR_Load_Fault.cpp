#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// ----- PARAMETRIZATION -----
// General grid parameters
Real nomFreq = 60;
Real nomOmega= nomFreq * 2 * PI;
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real initVoltAngle = -PI / 2;
Complex initTerminalVolt = nomPhPhVoltRMS * Complex(cos(initVoltAngle), sin(initVoltAngle));

//-----------Generator-----------//
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Real setPointActivePower=300e6;
Real mechPower = 300e6;
Real initActivePower = 300e6;
Real initReactivePower = 0;


void EMT_3ph_SynGen_Load(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchOpen,
	Real switchClosed, int SGModel, Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);
	
	// Nodes
	std::vector<Complex> initialVoltage_n1{ initTerminalVolt, 
											initTerminalVolt * SHIFT_TO_PHASE_B,
											initTerminalVolt * SHIFT_TO_PHASE_C
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
    Complex initComplexElectricalPower = Complex(initActivePower, initReactivePower);
    genEMT->setInitialValues(initComplexElectricalPower, mechPower, initTerminalVolt);

	auto load = CPS::EMT::Ph3::RXLoad::make("Load", logLevel);
	load->setParameters(Math::singlePhaseParameterToThreePhase(initActivePower/3), 
						Math::singlePhaseParameterToThreePhase(initReactivePower/3),
						nomPhPhVoltRMS);

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
		systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator3OrderVBR>(genEMT), load, fault});
	else if (SGModel==4)
		systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator4OrderVBR>(genEMT), load, fault});
	else if (SGModel==6)
		systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT},
			SystemComponentList{std::dynamic_pointer_cast<EMT::Ph3::SynchronGenerator6aOrderVBR>(genEMT), load, fault});
	else if (SGModel==7)
		systemEMT = SystemTopology(60,
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
	Real SwitchClosed = 0.1;
	Real SwitchOpen = 1e6;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 5;
	Real timeStep = 1e-3;
	int SGModel = 4;
	Real H = 3.7;
	std::string SGModel_str = "4Order";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.options["StepSize"];
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("SGModel") != args.options.end()) {
			SGModel = args.options["SGModel"];
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
			H = args.options["Inertia"];
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