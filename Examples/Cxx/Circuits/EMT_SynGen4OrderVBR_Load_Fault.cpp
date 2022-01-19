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


void EMT_3ph_SynGenTrStab_SteadyState(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchClosed, 
	Logger::Level logLevel) {

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
	auto genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make("SynGen", Logger::Level::debug);
	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, H,
	 	syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
		syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t,
		syngenKundur.Tq0_t); 
	Complex initComplexElectricalPower = Complex(initActivePower, initReactivePower);
	genEMT->setInitialValues(initComplexElectricalPower, mechPower, initTerminalVolt);

	auto load = CPS::EMT::Ph3::RXLoad::make("Load", logLevel);
	load->setParameters(Math::singlePhaseParameterToThreePhase(initActivePower/3), 
						Math::singlePhaseParameterToThreePhase(initReactivePower/3),
						nomPhPhVoltRMS);

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e12;
	fault->setParameters(Math::singlePhaseParameterToThreePhase(switchOpen), 
						 Math::singlePhaseParameterToThreePhase(switchClosed));
	fault->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	load->connect({ n1EMT });
	fault->connect({EMT::SimNode::GND, n1EMT});
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT},
			SystemComponentList{genEMT, load, fault});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->addAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->addAttribute("Etorque", 	genEMT->attribute("Etorque"));
    loggerEMT->addAttribute("delta", 	genEMT->attribute("delta"));
    loggerEMT->addAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->addAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	loggerEMT->addAttribute("Idq0", 	genEMT->attribute("Idq0"));
	loggerEMT->addAttribute("Edq0", 	genEMT->attribute("Edq0_t"));
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

	// Command line args processing
	CommandLineArgs args(argc, argv);
	Real SwitchClosed = 1e-3 * (24*24/555);
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real timeStep = 50e-6;
	Real H = 3.7;
	std::string rfault_str = "";
	std::string faultLength_str = "";
	std::string stepSize_str = "";
	std::string inertia_str = "";
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()){
			timeStep = args.options["StepSize"];
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("RSwitchClosed") != args.options.end()){
			SwitchClosed = args.options["RSwitchClosed"];
			rfault_str = "_RFault_" + std::to_string(SwitchClosed);
		}
		if (args.options.find("FaultLength") != args.options.end()){
			Real faultLength = args.options["FaultLength"];
			endTimeFault = startTimeFault + faultLength * 1e-3;
			faultLength_str = "_FaultLength_" + std::to_string(faultLength);
		}
		if (args.options.find("Inertia") != args.options.end()){
			H = args.options["Inertia"];
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	//Simultion parameters
	//Real logDownSampling = 1.0;
	Real logDownSampling = (100e-6) / timeStep;
	Real finalTime = 10;
	Logger::Level logLevel = Logger::Level::off;

	std::string simName ="EMT_SynGen4OrderVBR_Load_Fault" + rfault_str + faultLength_str + stepSize_str  + inertia_str;
	EMT_3ph_SynGenTrStab_SteadyState(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}