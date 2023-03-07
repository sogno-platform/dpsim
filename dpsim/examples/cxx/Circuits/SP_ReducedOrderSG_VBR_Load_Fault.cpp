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
	SynchronGeneratorFactory::SP::Ph1::registerSynchronGenerators();

	// Simulation parameters
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
		if (args.options.find("Inertia") != args.options.end())  {
			H = args.getOptionReal("Inertia");
			inertia_str = "_Inertia_" + std::to_string(H);
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
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "SP_SynGen" + SGModel + "Order_VBR_Load_Fault" + stepSize_str + inertia_str;


	// ----- Dynamic simulation ------
	String simNameSP = simName;
	Logger::setLogDir("logs/" + simNameSP);

	// Nodes
	auto initVoltN1 = std::vector<Complex>({
		Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
				GridParams.VnomMV * sin(GridParams.initVoltAngle))});
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initVoltN1);

	// Synchronous generator
	auto genSP = Factory<SP::Ph1::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genSP->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower,
				Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
						GridParams.VnomMV * sin(GridParams.initVoltAngle)));
	genSP->setModelAsNortonSource(true);

	// Exciter
	std::shared_ptr<Base::Exciter> exciterSP = nullptr;
	if (withExciter) {
		exciterSP = Factory<Base::Exciter>::get().create("DC1Simp", "Exciter", logLevel);
		exciterSP->setParameters(excitationEremia);
		genSP->addExciter(exciterSP);
	}

	// Turbine Governor
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorSP = nullptr;
	if (withTurbineGovernor) {
		turbineGovernorSP = Signal::TurbineGovernorType1::make("SynGen_TurbineGovernor", logLevel);
		turbineGovernorSP->setParameters(turbineGovernor.T3, turbineGovernor.T4,
			turbineGovernor.T5, turbineGovernor.Tc, turbineGovernor.Ts, turbineGovernor.R,
			turbineGovernor.Tmin, turbineGovernor.Tmax, turbineGovernor.OmegaRef);
		genSP->addGovernor(turbineGovernorSP);
	}

	// Load
	auto load = CPS::SP::Ph1::Load::make("Load", logLevel);
	load->setParameters(GridParams.initActivePower, GridParams.initReactivePower,
						GridParams.VnomMV);

	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", logLevel);
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genSP->connect({ n1SP });
	load->connect({ n1SP });
	fault->connect({SP::SimNode::GND, n1SP});
	auto systemSP = SystemTopology(GridParams.nomFreq,
							  SystemNodeList{n1SP},
							  SystemComponentList{genSP, load, fault});

	// Logging
	auto loggerSP = DataLogger::make(simNameSP, true, logDownSampling);
	loggerSP->logAttribute("v_gen", 	genSP->attribute("v_intf"));
    loggerSP->logAttribute("i_gen",		genSP->attribute("i_intf"));
    loggerSP->logAttribute("Te", 	 	genSP->attribute("Te"));
    loggerSP->logAttribute("delta", 	genSP->attribute("delta"));
    loggerSP->logAttribute("w_r", 		genSP->attribute("w_r"));
	loggerSP->logAttribute("Vdq0", 		genSP->attribute("Vdq0"));
	loggerSP->logAttribute("Idq0", 		genSP->attribute("Idq0"));
	loggerSP->logAttribute("Ef", 		genSP->attribute("Ef"));
	loggerSP->logAttribute("Tm",		genSP->attribute("Tm"));
	if (SGModel=="6a" || SGModel=="6b") {
		loggerSP->logAttribute("Edq0_s", 	 genSP->attribute("Edq_s"));
		loggerSP->logAttribute("Edq0_t", 	 genSP->attribute("Edq_t"));
	} else {
		loggerSP->logAttribute("Edq0", 		 genSP->attribute("Edq_t"));
	}

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
