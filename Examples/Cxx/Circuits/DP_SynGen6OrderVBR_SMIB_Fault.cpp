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

//-----------Generator-----------//
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Real setPointActivePower=300e6;
Real setPointVoltage=1.05*VnomMV;

// HV line parameters referred to MV side
Examples::Grids::CIGREHVAmerican::LineParameters lineCIGREHV;
Real lineLength = 100;
Real lineResistance = lineCIGREHV.lineResistancePerKm * lineLength * std::pow(ratio,2);
Real lineInductance = lineCIGREHV.lineReactancePerKm * lineLength * std::pow(ratio,2) / nomOmega;
Real lineCapacitance = lineCIGREHV.lineSusceptancePerKm * lineLength / std::pow(ratio,2) / nomOmega;
//Real lineConductance = 1e-15;
Real lineConductance = 8e-2;

void DP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchClosed,
	Logger::Level logLevel) {

	// ----- POWERFLOW FOR INITIALIZATION -----
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
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);
	
	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0), 0.0, 0.0};
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0), 0.0, 0.0};
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initialVoltage_n1);
	auto n2DP = SimNode<Complex>::make("n2DP", PhaseType::Single, initialVoltage_n2);

	// Components
	auto genDP = DP::Ph1::SynchronGenerator6aOrderVBR::make("SynGen", Logger::Level::debug);
	genDP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    genDP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", logLevel);
	//extnetDP->setParameters(VnomMV * RMS3PH_TO_PEAK1PH);
	extnetDP->setParameters(VnomMV);

    // Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", logLevel);
	lineDP->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	
	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e12;
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	fault->connect({SP::SimNode::GND, n1DP});
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{genDP, lineDP, extnetDP, fault});

	// Logging
	auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
	//loggerDP->addAttribute("v_slack", 	 extnetDP->attribute("v_intf"));
	//loggerDP->addAttribute("i_slack", 	 extnetDP->attribute("i_intf"));
	loggerDP->addAttribute("v_gen", 	 genDP->attribute("v_intf"));
    loggerDP->addAttribute("i_gen", 	 genDP->attribute("i_intf"));
    loggerDP->addAttribute("Etorque", 	 genDP->attribute("Etorque"));
    //loggerDP->addAttribute("delta", 	 genDP->attribute("delta"));
    //loggerDP->addAttribute("w_r", 		 genDP->attribute("w_r"));
	//loggerDP->addAttribute("Edq0_t",	 genDP->attribute("Edq_t"));
	//loggerDP->addAttribute("Edq0_s",	 genDP->attribute("Edq_s"));
	//loggerDP->addAttribute("Vdq0", 		 genDP->attribute("Vdq0"));
	//loggerDP->addAttribute("Idq0", 		 genDP->attribute("Idq0"));
	//loggerDP->addAttribute("Eabc", 		 genDP->attribute("Eabc"));

	Simulation simDP(simNameDP, logLevel);
	simDP.doInitFromNodesAndTerminals(true);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simDP.addLogger(loggerDP);
	simDP.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
	simDP.addEvent(sw1);

	auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
	simDP.addEvent(sw2);
	
	simDP.run();
}

int main(int argc, char* argv[]) {	

	// Command line args processing
	CommandLineArgs args(argc, argv);
	//Real SwitchClosed = 1e-3 * (24*24/555);
	Real SwitchClosed = 0.1;
	Real startTimeFault = 30.0;
	Real endTimeFault   = 30.1;
	Real timeStep = 400e-6;
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
	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Real finalTime = 40;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "DP_SynGen6OrderVBR_SMIB_Fault" + rfault_str + stepSize_str + faultLength_str + inertia_str;
	DP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}