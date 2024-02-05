#include <DPsim.h>
#include <dpsim-models/Factory.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
const Examples::Grids::SMIB::ReducedOrderSynchronGenerator::Scenario4::GridParams GridParams;

// Generator parameters
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;


int main(int argc, char* argv[]) {

	// initiaize gen factory
	SynchronGeneratorFactory::DP::Ph1::registerSynchronGenerators();

	//Simultion parameters
	Real startTimeFault = 30.0;
	Real endTimeFault   = 30.1;
	Real finalTime = 40;
	Real timeStep = 100e-6;
	Real H = syngenKundur.H;
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	std::string SGModel = "4";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("SGModel") != args.options.end()) {
			SGModel = args.getOptionString("SGModel");
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
	std::string simName = "DP_SynGen" + SGModel + "Order_VBR_SMIB_Fault" + stepSize_str + inertia_str;


	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, GridParams.VnomMV, GridParams.setPointActivePower,
						 GridParams.setPointVoltage, PowerflowBusType::PV);
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
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(0.1);
	simPF.setFinalTime(0.1);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- Dynamic simulation ------
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);

	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	auto initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0)};
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0)};
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initialVoltage_n1);
	auto n2DP = SimNode<Complex>::make("n2DP", PhaseType::Single, initialVoltage_n2);

	// Synchronous generator
	auto genDP = Factory<DP::Ph1::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
	genDP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s);
    genDP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genDP->setModelAsNortonSource(true);

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetDP->setParameters(GridParams.VnomMV);

    // Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", logLevel);
	lineDP->setParameters(GridParams.lineResistance, GridParams.lineInductance,
						  GridParams.lineCapacitance, GridParams.lineConductance);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	fault->connect({SP::SimNode::GND, n1DP});
	auto systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{genDP, lineDP, extnetDP, fault});

	// Logging
	auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
	loggerDP->logAttribute("v_gen", 	genDP->attribute("v_intf"));
	loggerDP->logAttribute("i_gen", 	genDP->attribute("i_intf"));
    loggerDP->logAttribute("Te", 	 	genDP->attribute("Te"));
    loggerDP->logAttribute("delta", 	genDP->attribute("delta"));
    loggerDP->logAttribute("w_r", 		genDP->attribute("w_r"));
	loggerDP->logAttribute("Edq0",		genDP->attribute("Edq0_t"));
	loggerDP->logAttribute("Vdq0", 		genDP->attribute("Vdq0"));
	loggerDP->logAttribute("Idq0", 		genDP->attribute("Idq0"));

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
