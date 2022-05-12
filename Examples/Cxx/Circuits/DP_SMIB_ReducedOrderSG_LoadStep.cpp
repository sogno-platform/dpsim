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
	String simName = "DP_SMIB_ReducedOrderSG_LoadStep";
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
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

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
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0)};
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initialVoltage_n1);
	auto n2DP = SimNode<Complex>::make("n2DP", PhaseType::Single, initialVoltage_n2);

	// Synchronous generator
	std::shared_ptr<DP::Ph1::SynchronGeneratorVBR> genDP = nullptr;
	if (sgType=="3") {
		genDP = DP::Ph1::SynchronGenerator3OrderVBR::make("SynGen", logLevel);
		std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator3OrderVBR>(genDP)->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage, 
			syngenKundur.nomFreq, H, 
			syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, syngenKundur.Ld_t, syngenKundur.Td0_t);
	} else if (sgType=="4") {
		genDP = DP::Ph1::SynchronGenerator4OrderVBR::make("SynGen", logLevel);
		std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator4OrderVBR>(genDP)->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage, 
			syngenKundur.nomFreq, H, 
			syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t); 
	} else if (sgType=="6b") {
		genDP = DP::Ph1::SynchronGenerator6bOrderVBR::make("SynGen", logLevel);
		std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator6bOrderVBR>(genDP)->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
	} else 
		throw CPS::SystemError("Unsupported reduced-order SG type!");	
    genDP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetDP->setParameters(gridParams.VnomMV);

    // Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", logLevel);
	lineDP->setParameters(gridParams.lineResistance, gridParams.lineInductance, 
						  gridParams.lineCapacitance, gridParams.lineConductance);
	
	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	SystemTopology systemDP;
	if (sgType=="3")
		systemDP = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator3OrderVBR>(genDP), lineDP, extnetDP});
	else if (sgType=="4")
		systemDP = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator4OrderVBR>(genDP), lineDP, extnetDP});
	else if (sgType=="6b")
		systemDP = SystemTopology(gridParams.nomFreq,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator6bOrderVBR>(genDP), lineDP, extnetDP});

	// Logging
	// log node voltage
	auto logger = DataLogger::make(simName, true, logDownSampling);
		for (auto node : systemDP.mNodes)
			logger->addAttribute(node->name() + ".V", node->attribute("v"));

	// log generator vars
	logger->addAttribute(genDP->name() + ".Tm", genDP->attribute("Tm"));
	logger->addAttribute(genDP->name() + ".Te", genDP->attribute("Te"));
	logger->addAttribute(genDP->name() + ".omega", genDP->attribute("w_r"));
	logger->addAttribute(genDP->name() + ".delta", genDP->attribute("delta"));

	// load step event
	std::shared_ptr<SwitchEvent> loadStepEvent = Examples::Events::createEventAddPowerConsumption("n1DP", std::round(loadStepEventTime/timeStep)*timeStep, gridParams.loadStepActivePower, systemDP, Domain::DP, logger);

	Simulation simDP(simNameDP, logLevel);
	simDP.doInitFromNodesAndTerminals(true);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simDP.addLogger(logger);
	simDP.doSystemMatrixRecomputation(true);

	// Events
	simDP.addEvent(loadStepEvent);
	
	simDP.run();
}