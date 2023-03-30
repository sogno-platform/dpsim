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

	// initiaize gen factory
	SynchronGeneratorFactory::DP::Ph1::registerSynchronGenerators();

	// Simulation parameters
	String simName = "DP_SMIB_ReducedOrderSGIterative_LoadStep";
	Real timeStep = 1e-3;
	Real finalTime = 35;

	// Default configuration
	Real loadStepEventTime = defaultConfig.loadStepEventTime;
	Real H = syngenKundur.H;
	Real tolerance = defaultConfig.tolerance;
	int maxIter = defaultConfig.maxIter;
	String SGModel = defaultConfig.sgType + "Iter";
	SGModel = "4TPM";	// options: "4PCM", "4TPM", "6PCM"

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("SimName") != args.options.end())
			simName = args.getOptionString("SimName");
		if (args.options.find("TimeStep") != args.options.end())
			timeStep = args.getOptionReal("TimeStep");
		if (args.options.find("Tolerance") != args.options.end())
			tolerance = args.getOptionReal("Tolerance");
		if (args.options.find("MaxIter") != args.options.end())
			maxIter = int(args.getOptionReal("MaxIter"));
		if (args.options.find("loadStepEventTime") != args.options.end())
			loadStepEventTime = args.getOptionReal("loadStepEventTime");
		if (args.options.find("inertia") != args.options.end())
			H = args.getOptionReal("inertia");
		if (args.options.find("SGModel") != args.options.end())
			SGModel = args.getOptionString("SGModel");
	}

	std::cout << "Simulation Parameters: " << std::endl;
	std::cout << "SimName: " << simName << std::endl;
	std::cout << "Time Step: " << timeStep << std::endl;
	std::cout << "Tolerance: " << tolerance << std::endl;
	std::cout << "Max N° of Iterations: " << maxIter << std::endl;
	std::cout << "SG: " << SGModel << std::endl;

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
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);

	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0)};
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initialVoltage_n1);
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0)};
	auto n2DP = SimNode<Complex>::make("n2DP", PhaseType::Single, initialVoltage_n2);

	// Synchronous generator
	auto genDP = Factory<CPS::Base::ReducedOrderSynchronGenerator<Complex>>::get().create(SGModel, "SynGen", logLevel);
	genDP->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, H,
		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
		syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
		syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genDP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genDP->setModelAsNortonSource(true);
	std::dynamic_pointer_cast<MNASyncGenInterface>(genDP)->setMaxIterations(maxIter);
	std::dynamic_pointer_cast<MNASyncGenInterface>(genDP)->setTolerance(tolerance);

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetDP->setParameters(gridParams.VnomMV);

    // Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", logLevel);
	lineDP->setParameters(gridParams.lineResistance,
	                      gridParams.lineInductance,
					      gridParams.lineCapacitance,
						  gridParams.lineConductance);

	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	SystemTopology systemDP = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{genDP, lineDP, extnetDP});

	// Logging
	auto logger = DataLogger::make(simName, true, logDownSampling);

	// log node voltage
	for (auto node : systemDP.mNodes)
		logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	logger->logAttribute(genDP->name() + ".Te", genDP->attribute("Te"));
	logger->logAttribute(genDP->name() + ".NIterations", genDP->attribute("NIterations"));
	logger->logAttribute(genDP->name() + ".Edq_t", genDP->attribute("Edq_t"));
	if (SGModel=="6PCM")
		logger->logAttribute(genDP->name() + ".Edq_s", genDP->attribute("Edq_s"));
	logger->logAttribute(genDP->name() + ".Vdq0", genDP->attribute("Vdq0"));
	logger->logAttribute(genDP->name() + ".Idq0", genDP->attribute("Idq0"));
	logger->logAttribute(genDP->name() + ".omega", genDP->attribute("w_r"));
	logger->logAttribute(genDP->name() + ".delta", genDP->attribute("delta"));
	logger->logAttribute(genDP->name() + ".Theta", genDP->attribute("Theta"));

	// load step event
	std::shared_ptr<SwitchEvent> loadStepEvent = Examples::Events::createEventAddPowerConsumption("n1DP", std::round(loadStepEventTime/timeStep)*timeStep, gridParams.loadStepActivePower, systemDP, Domain::DP, logger);

	Simulation simDP(simNameDP, logLevel);
	simDP.doInitFromNodesAndTerminals(true);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);
	simDP.addLogger(logger);

	// Events
	simDP.addEvent(loadStepEvent);

	simDP.run();
}
