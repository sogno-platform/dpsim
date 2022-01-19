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

void DP_1ph_SynGenTrStab_Fault(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchClosed,
	Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);
	
	// Nodes
	std::vector<Complex> initVoltN1 = std::vector<Complex>({
		Complex(nomPhPhVoltRMS * cos(initVoltAngle), nomPhPhVoltRMS * sin(initVoltAngle)),
		0.0, 0.0});
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initVoltN1);

	// Components
	auto genDP = DP::Ph1::SynchronGenerator4OrderVBR::make("SynGen", Logger::Level::debug);
	genDP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t,
			syngenKundur.Tq0_t); 
	Complex initComplexElectricalPower = Complex(initActivePower, initReactivePower);
    genDP->setInitialValues(initComplexElectricalPower, mechPower, 
		Complex(nomPhPhVoltRMS * cos(initVoltAngle), nomPhPhVoltRMS * sin(initVoltAngle)));

	auto load = CPS::DP::Ph1::RXLoad::make("Load", logLevel);
	load->setParameters(initActivePower, initReactivePower, nomPhPhVoltRMS);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
	Real switchOpen = 1e12;
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genDP->connect({ n1DP });
	load->connect({ n1DP });
	fault->connect({SP::SimNode::GND, n1DP});
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP},
			SystemComponentList{genDP, load, fault});

	// Logging
	auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
	loggerDP->addAttribute("v_gen", 	 genDP->attribute("v_intf"));
    loggerDP->addAttribute("i_gen", 	 genDP->attribute("i_intf"));
    loggerDP->addAttribute("Etorque", 	 genDP->attribute("Etorque"));
    loggerDP->addAttribute("delta", 	 genDP->attribute("delta"));
    loggerDP->addAttribute("w_r", 		 genDP->attribute("w_r"));
	loggerDP->addAttribute("Edq0",		 genDP->attribute("Edq0_t"));
	loggerDP->addAttribute("Vdq0", 		 genDP->attribute("Vdq0"));
	loggerDP->addAttribute("Idq0", 		 genDP->attribute("Idq0"));
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
	Real H = 3.7;
	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Real finalTime = 10;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "DP_SynGen4OrderVBR_Load_Fault" + rfault_str + stepSize_str + faultLength_str;
	DP_1ph_SynGenTrStab_Fault(simName, timeStep, finalTime, H,
		startTimeFault, endTimeFault, logDownSampling, SwitchClosed, logLevel);
}