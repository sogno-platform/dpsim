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
Real initVoltAngle = -PI / 2;

//-----------Generator-----------//
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
Real setPointActivePower=300e6;
Real mechPower = 300e6;
Real initActivePower = 300e6;
Real initReactivePower = 0;

//-----------Load-----------//
Real Rload = 1.92;

void SP_1ph_SynGen_Load(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchOpen,
	Real switchClosed, int SGModel, Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameSP = simName;
	Logger::setLogDir("logs/" + simNameSP);
	
	// Nodes
	std::vector<Complex> initVoltN1 = std::vector<Complex>({
		Complex(nomPhPhVoltRMS * cos(initVoltAngle), nomPhPhVoltRMS * sin(initVoltAngle)),
		0.0, 0.0});
	auto n1SP = SimNode<Complex>::make("n1SP", PhaseType::Single, initVoltN1);

	// Synchronous generator
	std::shared_ptr<SP::Ph1::SynchronGeneratorVBR> genSP = nullptr;
	if (SGModel==3)
		genSP = SP::Ph1::SynchronGenerator3OrderVBR::make("SynGen", logLevel);
	else if (SGModel==4)
		genSP = SP::Ph1::SynchronGenerator4OrderVBR::make("SynGen", logLevel);
	else if (SGModel==6)
		genSP = SP::Ph1::SynchronGenerator6aOrderVBR::make("SynGen", logLevel);
	else if (SGModel==7)
		genSP = SP::Ph1::SynchronGenerator6bOrderVBR::make("SynGen", logLevel);
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
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genSP->connect({ n1SP });
	load->connect({ n1SP });
	fault->connect({SP::SimNode::GND, n1SP});
	SystemTopology systemSP;
	if (SGModel==3)
		systemSP = SystemTopology(60,
			SystemNodeList{n1SP},
			SystemComponentList{std::dynamic_pointer_cast<SP::Ph1::SynchronGenerator3OrderVBR>(genSP), 
								load, fault});
	else if (SGModel==4)
		systemSP = SystemTopology(60,
			SystemNodeList{n1SP},
			SystemComponentList{std::dynamic_pointer_cast<SP::Ph1::SynchronGenerator4OrderVBR>(genSP), 
								load, fault});
	else if (SGModel==6)
		systemSP = SystemTopology(60,
			SystemNodeList{n1SP},
			SystemComponentList{std::dynamic_pointer_cast<SP::Ph1::SynchronGenerator6aOrderVBR>(genSP), 
								load, fault});
	else if (SGModel==7)
		systemSP = SystemTopology(60,
			SystemNodeList{n1SP},
			SystemComponentList{std::dynamic_pointer_cast<SP::Ph1::SynchronGenerator6bOrderVBR>(genSP), 
								load, fault});

	// Logging
	auto loggerSP = DataLogger::make(simNameSP, true, logDownSampling);
	loggerSP->addAttribute("v_gen", 	 genSP->attribute("v_intf"));
    loggerSP->addAttribute("i_gen", 	 genSP->attribute("i_intf"));
    loggerSP->addAttribute("Etorque", 	 genSP->attribute("Etorque"));
    //loggerSP->addAttribute("delta", 	 genSP->attribute("delta"));
    //loggerSP->addAttribute("w_r", 		 genSP->attribute("w_r"));
	//loggerSP->addAttribute("Edq0", 		 genSP->attribute("Edq0_t"));
	//loggerSP->addAttribute("Vdq0", 		 genSP->attribute("Vdq0"));
	//loggerSP->addAttribute("Idq0", 		 genSP->attribute("Idq0"));
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

	// Simulation parameters
	Real SwitchClosed = 0.1;
	Real SwitchOpen = 1e6;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 5;
	Real timeStep = 1e-3;
	Real H = 3.7;
	int SGModel = 4;
	std::string SGModel_str = "4Order";
	std::string stepSize_str = "";
	std::string inertia_str = "";

	// Command line args processing
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		if (args.options.find("StepSize") != args.options.end()) {
			timeStep = args.options["StepSize"];
			stepSize_str = "_StepSize_" + std::to_string(timeStep);
		}
		if (args.options.find("SGModel") != args.options.end()) {
			SGModel = args.options["SGModel"];
			if (SGModel==3)
				SGModel_str = "3Order";
			else if (SGModel==4)
				SGModel_str = "4Order";
			else if (SGModel==6)
				/// 6th order model (Marconato's model)
				SGModel_str = "6aOrder";
			else if (SGModel==7)
				/// 6th order model (Andorson-Fouad's model)
				SGModel_str = "6bOrder";
		}
		if (args.options.find("Inertia") != args.options.end())  {
			H = args.options["Inertia"];
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "SP_SynGen" + SGModel_str + "VBR_Load_Fault" + stepSize_str + inertia_str;
	SP_1ph_SynGen_Load(simName, timeStep, finalTime, H, startTimeFault, endTimeFault, 
			logDownSampling, SwitchOpen, SwitchClosed, SGModel, logLevel);
}