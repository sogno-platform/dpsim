#include <DPsim.h>
#include <dpsim-models/Factory.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
const Examples::Grids::SMIB::ScenarioConfig2 GridParams;

// Generator parameters
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

// Excitation system
const auto excitationParams = Examples::Components::Exciter::getExciterParametersEremia();

// PSS
const auto pssParams = Examples::Components::PowerSystemStabilizer::getPSS1AParametersPSAT();

// Turbine Goverour
const auto governorParams = Examples::Components::TurbineGovernor::getTurbineGovernorPSAT1();

int main(int argc, char* argv[]) {

	// initiaize factories
	ExciterFactory::registerExciters();
	SynchronGeneratorFactory::SP::Ph1::registerSynchronGenerators();

	// Simulation parameters
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 10;
	Real timeStep = 1e-3;
	Real H = syngenKundur.H;
	bool withExciter = false;
	bool withPSS = false;
	bool withTurbineGovernor = false;
	std::string SGModel = "4";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("SGModel") != args.options.end()) {
			SGModel = args.getOptionString("SGModel");
		}
		if (args.options.find("WithExciter") != args.options.end())  {
			withExciter = args.getOptionBool("WithExciter");
		}
		if (args.options.find("WithPSS") != args.options.end())  {
			withPSS = args.getOptionBool("WithPSS");
		}
		if (args.options.find("WithTurbineGovernor") != args.options.end())  {
			withTurbineGovernor = args.getOptionBool("WithTurbineGovernor");
		}
		if (args.options.find("Inertia") != args.options.end())  {
			H = args.getOptionReal("Inertia");
			inertia_str = "_Inertia_" + std::to_string(H);
		}
		if (args.options.find("FinalTime") != args.options.end())  {
			finalTime = args.getOptionReal("FinalTime");
		}
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
	}

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor(100e-6 / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::debug;
	std::string simName = "SP_SynGen" + SGModel + "Order_VBR_SMIB_Fault" + stepSize_str + inertia_str;


	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, GridParams.VnomMV,
						 GridParams.setPointActivePower, GridParams.setPointVoltage,
						 PowerflowBusType::PV);
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
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, logLevel);
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
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0)};
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0)};
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initialVoltage_n1);
	auto n2SP = SimNode<Complex>::make("n2SP", PhaseType::Single, initialVoltage_n2);

	// Synchronous generator
	auto genSP = Factory<SP::Ph1::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genSP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genSP->setModelAsNortonSource(true);

	// Exciter
	std::shared_ptr<Base::Exciter> exciterSP = nullptr;
	if (withExciter) {
		exciterSP = Factory<Base::Exciter>::get().create("DC1Simp", "Exciter", logLevel);
		exciterSP->setParameters(excitationParams);
		genSP->addExciter(exciterSP);
	}

	// Power system stabilizer
	std::shared_ptr<Signal::PSS1A> pssSP = nullptr;
	if (withPSS) {
		pssSP = Signal::PSS1A::make("SynGen_PSS", logLevel);
		pssSP->setParameters(pssParams);
		genSP->addPSS(pssSP);
	}

	// Turbine Governor
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorSP = nullptr;
	if (withTurbineGovernor) {
		turbineGovernorSP = Signal::TurbineGovernorType1::make("SynGen_TurbineGovernor", logLevel);
		turbineGovernorSP->setParameters(governorParams);
		genSP->addGovernor(turbineGovernorSP);
	}

	// Grid bus as Slack
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
	loggerSP->logAttribute("v_gen", genSP->attribute("v_intf"));
    loggerSP->logAttribute("i_gen", genSP->attribute("i_intf"));
    loggerSP->logAttribute("Te", 	genSP->attribute("Te"));
	loggerSP->logAttribute("Ef", 	genSP->attribute("Ef"));
    loggerSP->logAttribute("delta", genSP->attribute("delta"));
    loggerSP->logAttribute("w_r", 	genSP->attribute("w_r"));
	loggerSP->logAttribute("Vdq0", 	genSP->attribute("Vdq0"));
	loggerSP->logAttribute("Idq0",	genSP->attribute("Idq0"));
	if (SGModel=="5" || SGModel=="6a" || SGModel=="6b") {
		loggerSP->logAttribute("Edq0_s", 		 genSP->attribute("Edq_s"));
		loggerSP->logAttribute("Edq0_t", 		 genSP->attribute("Edq_t"));
	} else {
		loggerSP->logAttribute("Edq0", 		 genSP->attribute("Edq_t"));
	}
	loggerSP->logAttribute("v1", n1SP->attribute("v"));
	loggerSP->logAttribute("v2", n2SP->attribute("v"));

	// Turbine Governor
	if (withTurbineGovernor)
		loggerSP->logAttribute("Tm", turbineGovernorSP->attribute("Tm"));


	Simulation simSP(simNameSP, logLevel);
	simSP.doInitFromNodesAndTerminals(true);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.addLogger(loggerSP);
	simSP.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
	simSP.addEvent(sw1);

	auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
	simSP.addEvent(sw2);

	simSP.run();
}
