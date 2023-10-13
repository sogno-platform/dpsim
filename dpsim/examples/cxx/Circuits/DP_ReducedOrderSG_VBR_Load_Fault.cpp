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

// PSS
const auto pssPSAT = Examples::Components::PowerSystemStabilizer::getPSS1AParametersPSAT();

// Excitation system
const auto excitationEremia = Examples::Components::Exciter::getExciterParametersEremia();

// Turbine Governor Type 1
const auto governorPSAT1 = Examples::Components::TurbineGovernor::getTurbineGovernorPSAT1();

// Steam Turbine
//const Examples::Components::TurbineGovernor::SteamTurbine dSteamTurbine;
// Steam Turbine Governor
//const Examples::Components::TurbineGovernor::SteamTurbineGovernor dSteamGovernor;

// Hydro Turbine
//const Examples::Components::TurbineGovernor::HydroTurbine dHydroTurbine;
// Hydro Turbine Governor
//const Examples::Components::TurbineGovernor::HydroTurbineGovernor dHydroGovernor;

int main(int argc, char* argv[]) {

	// initiaize factories
	ExciterFactory::registerExciters();
	SynchronGeneratorFactory::DP::Ph1::registerSynchronGenerators();

	//Simultion parameters
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 30;
	Real finalTime = 60;
	Real timeStep = 1e-3;
	Real H = syngenKundur.H;
	bool withPSS = false;
	bool withExciter = false;
	bool withTurbineGovernor = false;
	//bool withExciter = true;
	//bool withTurbineGovernor = true;
	//bool hydro = true;
	//bool steam =false;
	//if ( hydro==steam)
	//	withTurbineGovernor=false;

	std::string SGModel = "4";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("SGModel") != args.options.end())
			SGModel = args.getOptionString("SGModel");
		if (args.options.find("WITHPSS") != args.options.end())
			withPSS = args.getOptionBool("WITHPSS");
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
	Logger::Level logLevel = Logger::Level::debug;
	std::string simName = "DP_SynGen" + SGModel + "Order_VBR_Load_Fault" + stepSize_str + inertia_str;


	// ----- Dynamic simulation ------
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);

	// Nodes
	auto initVoltN1 = std::vector<Complex>({
		Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
				GridParams.VnomMV * sin(GridParams.initVoltAngle))});
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initVoltN1);

	// Synchronous generator
	auto genDP = Factory<DP::Ph1::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genDP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genDP->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower,
			Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle),
					GridParams.VnomMV * sin(GridParams.initVoltAngle)));
	genDP->setModelAsNortonSource(true);

	// Exciter
	std::shared_ptr<Base::Exciter> exciterDP = nullptr;
	if (withExciter) {
		exciterDP = Factory<Base::Exciter>::get().create("DC1Simp", "Exciter", logLevel);
		exciterDP->setParameters(excitationEremia);
		genDP->addExciter(exciterDP);
	}

	// Power system stabilizer
	std::shared_ptr<Signal::PSS1A> pssDP = nullptr;
	if (withPSS) {
		pssDP = Signal::PSS1A::make("SynGen_PSS", logLevel);
		pssDP->setParameters(pssPSAT);
		genDP->addPSS(pssDP);
	}

	// Turbine Governor
	std::shared_ptr<Signal::TurbineGovernorType1> turbineGovernorDP = nullptr;
	if (withTurbineGovernor) {
		turbineGovernorDP = Signal::TurbineGovernorType1::make("SynGen_Governor", logLevel);
		turbineGovernorDP->setParameters(governorPSAT1);
		genDP->addGovernor(turbineGovernorDP);
	}

	/*
	if (withTurbineGovernor) {

	// Steam Turbine and Governor
	if(steam){
	std::shared_ptr<Signal::SteamTurbine> steamTurbine = nullptr;
		steamTurbine = Signal::SteamTurbine::make("SynGen_SteamTurbine", logLevel);
		steamTurbine->setParameters(dSteamTurbine.Fhp, dSteamTurbine.Fip,dSteamTurbine.Flp,
									dSteamTurbine.Tch, dSteamTurbine.Tco, dSteamTurbine.Trh);
		genDP->addSteamTurbine(steamTurbine);

		std::shared_ptr<Signal::SteamTurbineGovernor> steamTurbineGovernor = nullptr;
		steamTurbineGovernor = Signal::SteamTurbineGovernor::make("SynGen_SteamTurbineGovernor", logLevel);
		steamTurbineGovernor->setParameters(dSteamGovernor.OmRef, dSteamGovernor.R, dSteamGovernor.T2, dSteamGovernor.T3,  
								dSteamGovernor.delPmax, dSteamGovernor.delPmin, dSteamGovernor.Pmax, dSteamGovernor.Pmin);
		genDP->addSteamTurbineGovernor(steamTurbineGovernor);
	}
	
	// Hydro Turbine and Governor
	if(hydro){
		std::shared_ptr<Signal::HydroTurbine> hydroTurbine = nullptr;
		hydroTurbine = Signal::HydroTurbine::make("SynGen_HydroTurbine", logLevel);
		hydroTurbine->setParameters(dHydroTurbine.Tw);
		genDP->addHydroTurbine(hydroTurbine);

		std::shared_ptr<Signal::HydroTurbineGovernor> hydroTurbineGovernor = nullptr;
		hydroTurbineGovernor = Signal::HydroTurbineGovernor::make("SynGen_HydroTurbineGovernor", logLevel);
		hydroTurbineGovernor->setParameters(dHydroGovernor.OmRef, dHydroGovernor.R, dHydroGovernor.T1, dHydroGovernor.T2, dHydroGovernor.T3, dHydroGovernor.Pmax, dHydroGovernor.Pmin);
		genDP->addHydroTurbineGovernor(hydroTurbineGovernor);
	}
	}
	*/

	// Load
	auto load = CPS::DP::Ph1::RXLoad::make("Load", logLevel);
	load->setParameters(GridParams.initActivePower, GridParams.initReactivePower,
						GridParams.VnomMV);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genDP->connect({ n1DP });
	load->connect({ n1DP });
	fault->connect({SP::SimNode::GND, n1DP});
	auto systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP},
			SystemComponentList{genDP, load, fault});

	// Logging
	auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
	loggerDP->logAttribute("v_gen",	genDP->attribute("v_intf"));
    loggerDP->logAttribute("i_gen",	genDP->attribute("i_intf"));
    loggerDP->logAttribute("Te",	genDP->attribute("Te"));
    loggerDP->logAttribute("delta",	genDP->attribute("delta"));
    loggerDP->logAttribute("w_r",	genDP->attribute("w_r"));
	loggerDP->logAttribute("Vdq0",	genDP->attribute("Vdq0"));
	loggerDP->logAttribute("Idq0",	genDP->attribute("Idq0"));
	loggerDP->logAttribute("Ef",	genDP->attribute("Ef"));
	loggerDP->logAttribute("Tm", 	genDP->attribute("Tm"));

	Simulation simDP(simNameDP, logLevel);
	simDP.doInitFromNodesAndTerminals(true);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.addLogger(loggerDP);
	simDP.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
	simDP.addEvent(sw1);

	auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
	simDP.addEvent(sw2);

	simDP.run();
}
