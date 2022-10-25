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
<<<<<<< HEAD
=======

	// initiaize factories
	SynchronGeneratorFactory::EMT::Ph3::registerSynchronGenerators();
>>>>>>> 41541d9d (add PSS type 2 and add base class for exciter)

	// Simulation parameters
	String simName = "EMT_SMIB_ReducedOrderSG_LoadStep";
	Real timeStep = 10e-6;
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
	auto genEMT = Factory<EMT::Ph3::ReducedOrderSynchronGeneratorVBR>::get().create(sgType, "SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genEMT->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genEMT->setModelAsNortonSource(true);

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
	auto systemEMT = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, extnetEMT});

	// Logging
	// log node voltage
	auto logger = DataLogger::make(simName, true, logDownSampling);
		for (auto node : systemEMT.mNodes)
			logger->logAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	logger->logAttribute(genEMT->name() + ".Tm", genEMT->attribute("Tm"));
	logger->logAttribute(genEMT->name() + ".Te", genEMT->attribute("Te"));
	logger->logAttribute(genEMT->name() + ".omega", genEMT->attribute("w_r"));
	logger->logAttribute(genEMT->name() + ".delta", genEMT->attribute("delta"));

	// load step event
	std::shared_ptr<SwitchEvent3Ph> loadStepEvent = Examples::Events::createEventAddPowerConsumption3Ph("n1EMT", std::round(loadStepEventTime/timeStep)*timeStep, gridParams.loadStepActivePower, systemEMT, Domain::EMT, logger);

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(logger);
	simEMT.doSystemMatrixRecomputation(true);

	// Events
	simEMT.addEvent(loadStepEvent);

	simEMT.run();
}
