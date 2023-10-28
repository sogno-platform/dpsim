#include <DPsim.h>
#include <dpsim-models/Factory.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;
using namespace Examples::Grids::SMIB::ReducedOrderSynchronGenerator;

// Default configuration of scenario
Scenario6::Config defaultConfig;

// Grid parameters
Scenario6::GridParams gridParams;

// Generator parameters
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

int main(int argc, char* argv[]) {

	// initiaize factories
	SynchronGeneratorFactory::SP::Ph1::registerSynchronGenerators();

	// Simulation parameters
	String simName = "SP_SMIB_ReducedOrderSG_LoadStep";
	Real timeStep = 100e-6;
	Real finalTime = 35;

	// Default configuration
	String sgType = defaultConfig.sgType;
	Real loadStepEventTime = defaultConfig.loadStepEventTime;
	Real H = syngenKundur.H;

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("sgType") != args.options.end())
			sgType = args.getOptionString("sgType");
		if (args.options.find("loadStepEventTime") != args.options.end())
			loadStepEventTime = args.getOptionReal("loadStepEventTime");
		if (args.options.find("inertia") != args.options.end())
			H = args.getOptionReal("inertia");
	}

	// Configure logging
	Logger::Level logLevel = Logger::Level::info;

	// apply downsampling for simulation step sizes lower than 10us
	Real logDownSampling;
	if (timeStep < 10e-6)
		logDownSampling = floor((10e-6) / timeStep);
	else
		logDownSampling = 1.0;

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", logLevel);
	genPF->setParameters(syngenKundur.nomPower, gridParams.VnomMV, gridParams.setPointActivePower,
						 gridParams.setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(gridParams.VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetPF->setParameters(gridParams.VnomMV);
	extnetPF->setBaseVoltage(gridParams.VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", logLevel);
	linePF->setParameters(gridParams.lineResistance, gridParams.lineInductance,
						  gridParams.lineCapacitance, gridParams.lineConductance);
	linePF->setBaseVoltage(gridParams.VnomMV);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(gridParams.nomFreq,
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
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
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
	auto genSP = Factory<SP::Ph1::ReducedOrderSynchronGeneratorVBR>::get().create(sgType, "SynGen", logLevel);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genSP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genSP->setModelAsNortonSource(true);

	//Grid bus as Slack
	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetSP->setParameters(gridParams.VnomMV);

    // Line
	auto lineSP = SP::Ph1::PiLine::make("PiLine", logLevel);
	lineSP->setParameters(gridParams.lineResistance, gridParams.lineInductance,
						  gridParams.lineCapacitance, gridParams.lineConductance);

	// Topology
	genSP->connect({ n1SP });
	lineSP->connect({ n1SP, n2SP });
	extnetSP->connect({ n2SP });
	auto systemSP = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1SP, n2SP},
			SystemComponentList{genSP, lineSP, extnetSP});

	// Logging
	// log node voltage
	auto logger = DataLogger::make(simName, true, logDownSampling);
		for (auto node : systemSP.mNodes)
			logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	logger->logAttribute(genSP->name() + ".Tm", genSP->attribute("Tm"));
	logger->logAttribute(genSP->name() + ".Te", genSP->attribute("Te"));
	logger->logAttribute(genSP->name() + ".omega", genSP->attribute("w_r"));
	logger->logAttribute(genSP->name() + ".delta", genSP->attribute("delta"));

		// load step event
	std::shared_ptr<SwitchEvent> loadStepEvent = Examples::Events::createEventAddPowerConsumption("n1SP", std::round(loadStepEventTime/timeStep)*timeStep, gridParams.loadStepActivePower, systemSP, Domain::SP, logger);

	Simulation simSP(simNameSP, logLevel);
	simSP.doInitFromNodesAndTerminals(true);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.addLogger(logger);
	simSP.doSystemMatrixRecomputation(true);

	// Events
	simSP.addEvent(loadStepEvent);

	simSP.run();
}
