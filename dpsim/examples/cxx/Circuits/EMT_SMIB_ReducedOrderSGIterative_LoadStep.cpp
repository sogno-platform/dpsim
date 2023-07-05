#include <DPsim.h>
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

	// Simulation parameters
	String simName = "EMT_SMIB_ReducedOrderSGIterative_LoadStep";
	Real timeStep = 10e-6;
	Real finalTime = 35;
	Real timeStepPF = 0.1;
	Real finalTimePF = 0.1;

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

	// set solver parameters
	auto solverParameters = std::make_shared<SolverParametersMNA>();
	solverParameters->setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	solverParameters->setInitFromNodesAndTerminals(false);

	// Simulation
	Simulation simPF(simNamePF, logLevel);
	simPF.setSystem(systemPF);
	simPF.setSimulationParameters(timeStepPF, finalTimePF);
	simPF.setSolverParameters(Domain::SP, Solver::Type::NRP, solverParameters);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/" + simNameEMT);

	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0),
											n1PF->voltage()(0,0) * SHIFT_TO_PHASE_B,
											n1PF->voltage()(0,0) * SHIFT_TO_PHASE_C
										  };
	auto n1EMT = SimNode<Real>::make("n1EMT", PhaseType::ABC, initialVoltage_n1);
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0),
											n2PF->voltage()(0,0) * SHIFT_TO_PHASE_B,
											n2PF->voltage()(0,0) * SHIFT_TO_PHASE_C
										  };
	auto n2EMT = SimNode<Real>::make("n2EMT", PhaseType::ABC, initialVoltage_n2);

	// Synchronous generator
	auto genEMT = EMT::Ph3::SynchronGenerator4OrderPCM::make("SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, H,
		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
		syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t);
    genEMT->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genEMT->setMaxIterations(maxIter);
	genEMT->setTolerance(tolerance);

	//Grid bus as Slack
	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", logLevel);

    // Line
	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", logLevel);
	lineEMT->setParameters(Math::singlePhaseParameterToThreePhase(gridParams.lineResistance),
	                      Math::singlePhaseParameterToThreePhase(gridParams.lineInductance),
					      Math::singlePhaseParameterToThreePhase(gridParams.lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(gridParams.lineConductance));

	// Topology
	genEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	extnetEMT->connect({ n2EMT });
	SystemTopology systemEMT = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, extnetEMT});

	// Logging
	// log node voltage
	auto logger = DataLogger::make(simName, true, logDownSampling);
		for (auto node : systemEMT.mNodes)
			logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	//logger->logAttribute(genEMT->name() + ".Tm", genEMT->attribute("Tm"));
	logger->logAttribute(genEMT->name() + ".Te", genEMT->attribute("Te"));
	logger->logAttribute(genEMT->name() + ".omega", genEMT->attribute("w_r"));
	logger->logAttribute(genEMT->name() + ".delta", genEMT->attribute("delta"));
	logger->logAttribute(genEMT->name() + ".NIterations", genEMT->attribute("NIterations"));
	//logger->logAttribute(genEMT->name() + ".theta", genEMT->attribute("Theta"));

	// load step event
	std::shared_ptr<SwitchEvent3Ph> loadStepEvent = Examples::Events::createEventAddPowerConsumption3Ph("n1EMT", std::round(loadStepEventTime/timeStep)*timeStep, gridParams.loadStepActivePower, systemEMT, Domain::EMT, logger);

	// set solver parameters
	auto solverParameterEMT = std::make_shared<SolverParametersMNA>();
	solverParameterEMT->setInitFromNodesAndTerminals(true);
	solverParameterEMT->setDirectLinearSolverImplementation(CPS::DirectLinearSolverImpl::SparseLU);

	//
	Simulation simEMT(simNameEMT, logLevel);
	simEMT.setSystem(systemEMT);
	simEMT.setSimulationParameters(timeStep, finalTime);
	simEMT.setSolverParameters(Domain::EMT, Solver::Type::MNA, solverParameterEMT);
	simEMT.addLogger(logger);

	// Events
	simEMT.addEvent(loadStepEvent);

	simEMT.run();
}
