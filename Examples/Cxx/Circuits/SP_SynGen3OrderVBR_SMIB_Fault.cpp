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
Real lineConductance = 1e-15;
//Real lineConductance = 8e-2;

void SP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, Real H,
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
	String simNameSP = simName;
	Logger::setLogDir("logs/" + simNameSP);
	
	// Extract relevant powerflow results
	Real initActivePower = genPF->getApparentPower().real();
	Real initReactivePower = genPF->getApparentPower().imag();
	Complex initElecPower = Complex(initActivePower, initReactivePower);
	Real initMechPower = initActivePower;

	// Nodes
	std::vector<Complex> initialVoltage_n1{ n1PF->voltage()(0,0), 0.0, 0.0};
	std::vector<Complex> initialVoltage_n2{ n2PF->voltage()(0,0), 0.0, 0.0};
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initialVoltage_n1);
	auto n2SP = SimNode<Complex>::make("n2SP", PhaseType::Single, initialVoltage_n2);

	// Components
	auto genSP = SP::Ph1::SynchronGenerator3OrderVBR::make("SynGen", Logger::Level::debug);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    genSP->setInitialValues(initElecPower, initMechPower, n1PF->voltage()(0,0));

	//Grid bus as Slack
	auto extnetSP = SP::Ph1::NetworkInjection::make("Slack", logLevel);
	extnetSP->setParameters(VnomMV, nomFreq);

    // Line
	auto lineSP = SP::Ph1::PiLine::make("PiLine", logLevel);
	lineSP->setParameters(lineResistance, lineInductance, lineCapacitance, lineConductance);
	
	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e12;
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genSP->connect({ n1SP });
	lineSP->connect({ n1SP, n2SP });
	extnetSP->connect({ n2SP });
	fault->connect({SP::SimNode::GND, n1SP});
	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP, n2SP},
			SystemComponentList{genSP, lineSP, extnetSP, fault});

	// Logging
	auto loggerSP = DataLogger::make(simNameSP, true, logDownSampling);
	loggerSP->addAttribute("v_slack", 	 extnetSP->attribute("v_intf"));
	loggerSP->addAttribute("i_slack", 	 extnetSP->attribute("i_intf"));
	loggerSP->addAttribute("v_gen", 	 genSP->attribute("v_intf"));
    loggerSP->addAttribute("i_gen", 	 genSP->attribute("i_intf"));
    loggerSP->addAttribute("Etorque", 	 genSP->attribute("Etorque"));
    loggerSP->addAttribute("delta", 	 genSP->attribute("delta"));
    loggerSP->addAttribute("w_r", 		 genSP->attribute("w_r"));
	loggerSP->addAttribute("Edq0", 		 genSP->attribute("Edq_t"));
	loggerSP->addAttribute("Vdq0", 		 genSP->attribute("Vdq0"));
	loggerSP->addAttribute("Idq0", 		 genSP->attribute("Idq0"));
	//loggerSP->addAttribute("Evbr", 		 genSP->attribute("Evbr"));

	Simulation simSP(simNameSP, logLevel);
	simSP.doInitFromNodesAndTerminals(true);
	simSP.setSystem(systemSP);
	simSP.setTimeStep(timeStep);
	simSP.setFinalTime(finalTime);
	simSP.setDomain(Domain::SP);
	simSP.setMnaSolverImplementation(DPsim::MnaSolverFactory::EigenSparse);
	simSP.addLogger(loggerSP);
	simSP.doSystemMatrixRecomputation(true);

	// Events
	auto sw1 = SwitchEvent::make(startTimeFault, fault, true);
	simSP.addEvent(sw1);

	auto sw2 = SwitchEvent::make(endTimeFault, fault, false);
	simSP.addEvent(sw2);
	
	simSP.run();
}

int main(int argc, char* argv[]) {	

	// Command line args processing
	CommandLineArgs args(argc, argv);
	Real SwitchClosed = 1e-3 * (24*24/555);
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real timeStep = 1e-3;
	std::string rfault_str = "";
	std::string faultLength_str = "";
	std::string stepSize_str = "";
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
	}

	//Simultion parameters
	//Real H = 1.85;
	Real H = 3.7;
	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Real finalTime = 20;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "SP_SynGen3OrderVBR_SMIB_Fault" + rfault_str + stepSize_str + faultLength_str;
	SP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}