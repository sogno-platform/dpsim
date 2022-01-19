#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// ----- PARAMETRIZATION -----
// General grid parameters
Real nomFreq = 60;
Real nomOmega= nomFreq * 2 * PI;
Real nomPower = 555e6;
Real nomPhPhVoltRMS = 24e3;
Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
Real initVoltAngle = -PI / 2;

//-----------Generator-----------//
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Real setPointActivePower=300e6;
Real mechPower = 300e6;
Real initActivePower = 300e6;
Real initReactivePower = 0;

//-----------Load-----------//
Real Rload = 1.92;

void SP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchClosed, 
	Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameSP = simName;
	Logger::setLogDir("logs/" + simNameSP);
	
	// Nodes
	std::vector<Complex> initVoltN1 = std::vector<Complex>({
		Complex(nomPhPhVoltRMS * cos(initVoltAngle), nomPhPhVoltRMS * sin(initVoltAngle)),
		0.0, 0.0});
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initVoltN1);

	// Components
	auto genSP = SP::Ph1::SynchronGenerator4OrderVBR::make("SynGen", Logger::Level::debug);
	genSP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
	Complex initComplexElectricalPower = Complex(initActivePower, initReactivePower);
    genSP->setInitialValues(initComplexElectricalPower, mechPower, 
		Complex(nomPhPhVoltRMS * cos(initVoltAngle), nomPhPhVoltRMS * sin(initVoltAngle)));

	auto load = CPS::SP::Ph1::Load::make("Load", logLevel);
	load->setParameters(initActivePower, initReactivePower, nomPhPhVoltRMS);
	
	//Breaker
	auto fault = CPS::SP::Ph1::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e12;
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genSP->connect({ n1SP });
	load->connect({ n1SP });
	fault->connect({SP::SimNode::GND, n1SP});
	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP},
			SystemComponentList{genSP, load, fault});

	// Logging
	auto loggerSP = DataLogger::make(simNameSP, true, logDownSampling);
	loggerSP->addAttribute("v_gen", 	 genSP->attribute("v_intf"));
    loggerSP->addAttribute("i_gen", 	 genSP->attribute("i_intf"));
    loggerSP->addAttribute("Etorque", 	 genSP->attribute("Etorque"));
    loggerSP->addAttribute("delta", 	 genSP->attribute("delta"));
    loggerSP->addAttribute("w_r", 		 genSP->attribute("w_r"));
	loggerSP->addAttribute("Edq0", 		 genSP->attribute("Edq0_t"));
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
	Real timeStep = 100e-6;
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
	Real H =3.7;
	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Real finalTime = 10;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "SP_SynGen4OrderVBR_Load_Fault" + rfault_str + stepSize_str + faultLength_str;
	SP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}