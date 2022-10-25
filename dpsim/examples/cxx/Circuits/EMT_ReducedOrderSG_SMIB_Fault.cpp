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
	SynchronGeneratorFactory::EMT::Ph3::registerSynchronGenerators();

	// Simultion parameters
	Real switchClosed = GridParams.SwitchClosed;
	Real switchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 30.0;
	Real endTimeFault   = 30.1;
	Real finalTime = 40;
	Real timeStep = 100e-6;
	Real H = syngenKundur.H;
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
	std::string simName ="EMT_SynGen_" + SGModel + "Order_VBR_SMIB_Fault" + stepSize_str + inertia_str;



	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	// Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(syngenKundur.nomPower, GridParams.VnomMV, GridParams.setPointActivePower,
						 GridParams.setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(GridParams.VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	// Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(GridParams.VnomMV);
	extnetPF->setBaseVoltage(GridParams.VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	// Line
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
	Simulation simPF(simNamePF, Logger::Level::off);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(0.1);
	simPF.setFinalTime(0.1);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();


	// ----- Dynamic simulation ------
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);

	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	auto initElecPower = Complex(initActivePower, initReactivePower);
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
	auto genEMT = Factory<EMT::Ph3::ReducedOrderSynchronGeneratorVBR>::get().create(SGModel, "SynGen", logLevel);
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
	lineEMT->setParameters(Math::singlePhaseParameterToThreePhase(GridParams.lineResistance),
	                      Math::singlePhaseParameterToThreePhase(GridParams.lineInductance),
					      Math::singlePhaseParameterToThreePhase(GridParams.lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(GridParams.lineConductance));

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", logLevel);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(switchOpen),
						 Math::singlePhaseParameterToThreePhase(switchClosed));
	fault->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	extnetEMT->connect({ n2EMT });
	fault->connect({EMT::SimNode::GND, n1EMT});

	auto systemEMT = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, fault, extnetEMT});

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Te", 	genEMT->attribute("Te"));
    loggerEMT->logAttribute("delta", 	genEMT->attribute("delta"));
    loggerEMT->logAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->logAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	loggerEMT->logAttribute("Idq0", 	genEMT->attribute("Idq0"));
	loggerEMT->logAttribute("Edq0", 	genEMT->attribute("Edq0_t"));

	// Configure simulation
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
}
