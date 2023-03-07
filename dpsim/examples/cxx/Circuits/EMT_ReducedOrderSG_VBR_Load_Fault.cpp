#include <DPsim.h>
#include <dpsim-models/Factory.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
const Examples::Grids::SMIB::ScenarioConfig3 GridParams;

// Generator parameters
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

// Excitation system
const Base::ExciterParameters excitationEremia = Examples::Components::Exciter::getExciterEremia();

// Turbine Goverour
const Examples::Components::TurbineGovernor::TurbineGovernorPSAT1 turbineGovernor;

int main(int argc, char* argv[]) {

	// initiaize factories
	ExciterFactory::registerExciters();
	SynchronGeneratorFactory::EMT::Ph3::registerSynchronGenerators();

	// Simultion parameters
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 5;
	Real timeStep = 1e-3;
	Real H = syngenKundur.H;
	bool withExciter = false;
	bool withTurbineGovernor = false;
	std::string SGModel = "4";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("SGModel") != args.options.end())
			SGModel = args.getOptionString("SGModel");
		if (args.options.find("WITHEXCITER") != args.options.end())
			withExciter = args.getOptionBool("WITHEXCITER");
		if (args.options.find("WithTurbineGovernor") != args.options.end())
			withTurbineGovernor = args.getOptionBool("WithTurbineGovernor");
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
		logDownSampling = floor(100e-6 / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName ="EMT_SynGen" + SGModel + "Order_VBR_Load_Fault" + stepSize_str + inertia_str;


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
	auto genEMT = Factory<EMT::Ph3::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genEMT->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower,
							 GridParams.initTerminalVolt);
	genEMT->setModelAsNortonSource(true);

	// Exciter
	std::shared_ptr<Base::Exciter> exciterEMT = nullptr;
	if (withExciter) {
		exciterEMT = Factory<Base::Exciter>::get().create("DC1Simp", "Exciter", logLevel);
		exciterEMT->setParameters(excitationEremia);
		genEMT->addExciter(exciterEMT);
	}

	// Turbine Governor
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorEMT = nullptr;
	if (withTurbineGovernor) {
		turbineGovernorEMT = Signal::TurbineGovernorType1::make("SynGen_TurbineGovernor", logLevel);
		turbineGovernorEMT->setParameters(turbineGovernor.T3, turbineGovernor.T4,
			turbineGovernor.T5, turbineGovernor.Tc, turbineGovernor.Ts, turbineGovernor.R,
			turbineGovernor.Tmin, turbineGovernor.Tmax, turbineGovernor.OmegaRef);
		genEMT->addGovernor(turbineGovernorEMT);
	}

	// Load
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

	auto systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT},
			SystemComponentList{genEMT, load, fault});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Te",		genEMT->attribute("Te"));
    loggerEMT->logAttribute("delta", 	genEMT->attribute("delta"));
    loggerEMT->logAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->logAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	loggerEMT->logAttribute("Idq0", 	genEMT->attribute("Idq0"));
	loggerEMT->logAttribute("Ef",   	genEMT->attribute("Ef"));
	loggerEMT->logAttribute("Tm", 		genEMT->attribute("Tm"));

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
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
