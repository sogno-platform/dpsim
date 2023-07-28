#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// ----- PARAMETRIZATION -----
// General grid parameters
Real nomPower = 555e6;
Real VnomMV = 24e3;
Real VnomHV = 230e3;
Real nomFreq = 60;
Real ratio = VnomMV/VnomHV;
Real nomOmega= nomFreq * 2 * PI;

// Generator
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Real setPointActivePower=300e6;
Real setPointVoltage=1.05*VnomMV;

// HV line parameters referred to MV side
Examples::Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
Real lineLength = 100;
Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength*std::pow(ratio,2);
Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength*std::pow(ratio,2) / nomOmega;
Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength/std::pow(ratio,2) / nomOmega;
Real lineConductance = 8e-2;

void EMT_3ph_4OrderSynGenIter(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real switchClosed, Real logDownSampling,
	Real maxIterations, Real tolerance, Logger::Level logLevel) {

	//  // ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(nomPower, VnomMV, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(VnomMV);
	extnetPF->setBaseVoltage(VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	linePF->setBaseVoltage(VnomMV);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = CPS::DataLogger::make(simNamePF);
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
	String simNameEMT = simName;
	Logger::setLogDir("logs/"+simNameEMT);

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

	// Components
	auto genEMT = EMT::Ph3::SynchronGenerator4OrderPCM::make("SynGen", logLevel);
	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, H,
	 	syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll,
		syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t,
		syngenKundur.Tq0_t);
    genEMT->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));
	genEMT->setMaxIterations(maxIterations);
	genEMT->setTolerance(tolerance);

	//Grid bus as Slack
	auto extnetEMT = EMT::Ph3::NetworkInjection::make("Slack", logLevel);

    // Line
	auto lineEMT = EMT::Ph3::PiLine::make("PiLine", logLevel);
	lineEMT->setParameters(Math::singlePhaseParameterToThreePhase(lineResistance),
	                      Math::singlePhaseParameterToThreePhase(lineInductance),
					      Math::singlePhaseParameterToThreePhase(lineCapacitance),
						  Math::singlePhaseParameterToThreePhase(lineConductance));

	//Breaker
	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e6;
	fault->setParameters(Math::singlePhaseParameterToThreePhase(switchOpen),
						 Math::singlePhaseParameterToThreePhase(switchClosed));
	fault->openSwitch();

	// Topology
	genEMT->connect({ n1EMT });
	lineEMT->connect({ n1EMT, n2EMT });
	extnetEMT->connect({ n2EMT });
	fault->connect({EMT::SimNode::GND, n1EMT});
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT},
			SystemComponentList{genEMT, lineEMT, fault, extnetEMT});

	// Logging
	auto loggerEMT = CPS::DataLogger::make(simNameEMT, true, logDownSampling);
	//loggerEMT->logAttribute("v2", n2EMT->attribute("v"));
	//loggerEMT->logAttribute("v_gen", 	genEMT->attribute("v_intf"));
    //loggerEMT->logAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->logAttribute("Etorque", 	genEMT->attribute("Te"));
    loggerEMT->logAttribute("delta", 	genEMT->attribute("delta"));
    loggerEMT->logAttribute("w_r", 		genEMT->attribute("w_r"));
	loggerEMT->logAttribute("Vdq0", 		genEMT->attribute("Vdq0"));
	loggerEMT->logAttribute("Idq0", 		genEMT->attribute("Idq0"));
	loggerEMT->logAttribute("Edq0", 	genEMT->attribute("Edq0_t"));
	loggerEMT->logAttribute("NIterations", 	genEMT->attribute("NIterations"));

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.setDirectLinearSolverImplementation(DPsim::DirectLinearSolverImpl::SparseLU);

	// Events
	auto sw1 = SwitchEvent3Ph::make(startTimeFault, fault, true);
	simEMT.addEvent(sw1);

	auto sw2 = SwitchEvent3Ph::make(endTimeFault, fault, false);
	simEMT.addEvent(sw2);

	simEMT.run();
	simEMT.logStepTimes(simNameEMT + "_step_times");
}

int main(int argc, char* argv[]) {

	// Command line args processing
	CommandLineArgs args(argc, argv);
	Real switchClosed = 0.001;
	Real iteration = 20;
	Real tolerance = 1e-6;
	Real timeStep = 10e-6;

	std::string stepSize_str = "";
	std::string iteration_str = "";
	std::string tolerance_str = "";
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()){
			timeStep = args.getOptionReal("StepSize");
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("MaxIter") != args.options.end()){
			iteration = args.getOptionReal("MaxIter");
			iteration_str = "_MaxIter_" + std::to_string(iteration);
		}
		if (args.options.find("Tol") != args.options.end()){
			tolerance = args.getOptionReal("Tol");
			tolerance_str = "_Tolerance_" + std::to_string(tolerance*1e6);
		}
	}

	//Simultion parameters
	Real H = 3.7;
	Real finalTime = 10.0;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	std::string simName ="EMT_SynGen4OrderIter_SMIB_Fault" + stepSize_str + tolerance_str + iteration_str;

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::off;


	EMT_3ph_4OrderSynGenIter(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, switchClosed, logDownSampling,
		iteration, tolerance, logLevel);
}
