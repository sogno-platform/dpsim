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


void EMT_3ph_SynGenTrStab_SteadyState(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchClosed, 
	Logger::Level logLevel) {

	// ----- POWERFLOW FOR INITIALIZATION -----
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	// Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	genPF->setParameters(nomPower, VnomMV, setPointActivePower, setPointVoltage, PowerflowBusType::PV);
    genPF->setBaseVoltage(VnomMV);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	// Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(VnomMV);
	extnetPF->setBaseVoltage(VnomMV);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);
	
	// Line
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
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

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
	
	auto genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make("SynGen", Logger::Level::debug);

	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, H,
	 	syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
		syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t,
		syngenKundur.Tq0_t); 
	/*
	// VBR Simple 
	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage,
		syngenKundur.nomFreq, syngenKundur.H, Kd, 
 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
		syngenKundur.Lmd, syngenKundur.Lmq, syngenKundur.Llfd, 
		syngenKundur.Llkq1, syngenKundur.Rs, syngenKundur.Rfd, syngenKundur.Rkq1);
	genEMT->setNumericalMethod(numericalMethod);
	// VBR Full 
	genEMT->setOperationalParametersPerUnit(
		syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFieldCurr,
		syngenKundur.nomFreq, syngenKundur.H, Kd, 
 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
		syngenKundur.Lmd, syngenKundur.Lmq, syngenKundur.Llfd, 
		syngenKundur.Llkq1, syngenKundur.Rs, syngenKundur.Rfd, syngenKundur.Rkq1);
	*/
	genEMT->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));


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
	auto loggerEMT = DataLogger::make(simNameEMT, true, logDownSampling);
	//loggerEMT->addAttribute("i_slack", 	extnetEMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_gen", 	genEMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen", 	genEMT->attribute("i_intf"));
    loggerEMT->addAttribute("Etorque", 	genEMT->attribute("Etorque"));
    //loggerEMT->addAttribute("delta", 	genEMT->attribute("delta"));
    //loggerEMT->addAttribute("w_r", 	genEMT->attribute("w_r"));
	//loggerEMT->addAttribute("Vdq0", 	genEMT->attribute("Vdq0"));
	//loggerEMT->addAttribute("Idq0", 	genEMT->attribute("Idq0"));
	//loggerEMT->addAttribute("Edq0", 	genEMT->attribute("Edq0_t"));
	//loggerEMT->addAttribute("Evbr", 	genEMT->attribute("Evbr"));

	Simulation simEMT(simNameEMT, logLevel);
	simEMT.doInitFromNodesAndTerminals(true);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true);

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
	//Real SwitchClosed = 1e-3 * (24*24/555);
	Real SwitchClosed = 0.1;
	Real startTimeFault = 30.0;
	Real endTimeFault   = 30.1;
	Real timeStep = 1e-6;
	Real H = 3.7;
	std::string rfault_str = "";
	std::string faultLength_str = "";
	std::string stepSize_str = "";
	std::string inertia_str = "";
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()){
			timeStep = args.options["StepSize"];
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("RSwitchClosed") != args.options.end()){
			SwitchClosed = args.options["RSwitchClosed"];
			rfault_str = "_RFault_" + std::to_string(SwitchClosed);
		}
		if (args.options.find("FaultLength") != args.options.end()){
			Real faultLength = args.options["FaultLength"];
			endTimeFault = startTimeFault + faultLength * 1e-3;
			faultLength_str = "_FaultLength_" + std::to_string(faultLength);
		}
		if (args.options.find("Inertia") != args.options.end()){
			H = args.options["Inertia"];
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	//Simultion parameters
	//Real logDownSampling = 1.0;
	Real logDownSampling = (100e-6) / timeStep;
	Real finalTime = 40;
	Logger::Level logLevel = Logger::Level::off;

	std::string simName ="EMT_SynGen4OrderVBR_SMIB_Fault" + rfault_str + faultLength_str + stepSize_str + inertia_str;
	EMT_3ph_SynGenTrStab_SteadyState(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}