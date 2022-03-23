#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

// Grid parameters
Examples::Grids::SMIB::ScenarioConfig3 GridParams;

// Generator parameters
Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;


void DP_1ph_SynGen_Fault(String simName, Real timeStep, Real finalTime, Real H,
	Real startTimeFault, Real endTimeFault, Real logDownSampling, Real switchOpen,
	Real switchClosed, int SGModel, Logger::Level logLevel) {

	// ----- Dynamic simulation ------
	String simNameDP = simName;
	Logger::setLogDir("logs/" + simNameDP);
	
	// Nodes
	std::vector<Complex> initVoltN1 = std::vector<Complex>({
		Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle), 
				GridParams.VnomMV * sin(GridParams.initVoltAngle))});
	auto n1DP = SimNode<Complex>::make("n1DP", PhaseType::Single, initVoltN1);

	// Synchronous generator
	std::shared_ptr<DP::Ph1::SynchronGeneratorVBR> genDP = nullptr;
	if (SGModel==3)
		genDP = DP::Ph1::SynchronGenerator3OrderVBR::make("SynGen", logLevel);
	else if (SGModel==4)
		genDP = DP::Ph1::SynchronGenerator4OrderVBR::make("SynGen", logLevel);
	else if (SGModel==6)
		genDP = DP::Ph1::SynchronGenerator6aOrderVBR::make("SynGen", logLevel);
	else if (SGModel==7)
		genDP = DP::Ph1::SynchronGenerator6bOrderVBR::make("SynGen", logLevel);

	genDP->setOperationalParametersPerUnit(
			syngenKundur.nomPower, syngenKundur.nomVoltage,
			syngenKundur.nomFreq, H,
	 		syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Ll, 
			syngenKundur.Ld_t, syngenKundur.Lq_t, syngenKundur.Td0_t, syngenKundur.Tq0_t,
			syngenKundur.Ld_s, syngenKundur.Lq_s, syngenKundur.Td0_s, syngenKundur.Tq0_s); 
    
    genDP->setInitialValues(GridParams.initComplexElectricalPower, GridParams.mechPower, 
			Complex(GridParams.VnomMV * cos(GridParams.initVoltAngle), 
					GridParams.VnomMV * sin(GridParams.initVoltAngle)));

	auto load = CPS::DP::Ph1::RXLoad::make("Load", logLevel);
	load->setParameters(GridParams.initActivePower, GridParams.initReactivePower, 
						GridParams.VnomMV);

	//Breaker
	auto fault = CPS::DP::Ph1::Switch::make("Br_fault", logLevel);
	fault->setParameters(switchOpen, switchClosed);
	fault->open();

	// Topology
	genDP->connect({ n1DP });
	load->connect({ n1DP });
	fault->connect({SP::SimNode::GND, n1DP});
	SystemTopology systemDP;
	if (SGModel==3)
		systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator3OrderVBR>(genDP), load, fault});
	else if (SGModel==4)
		systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator4OrderVBR>(genDP), load, fault});
	else if (SGModel==6)
		systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator6aOrderVBR>(genDP), load, fault});
	else if (SGModel==7)
		systemDP = SystemTopology(GridParams.nomFreq,
			SystemNodeList{n1DP},
			SystemComponentList{std::dynamic_pointer_cast<DP::Ph1::SynchronGenerator6bOrderVBR>(genDP), load, fault});

	// Logging
	auto loggerDP = DataLogger::make(simNameDP, true, logDownSampling);
	loggerDP->addAttribute("v_gen", 	 genDP->attribute("v_intf"));
    loggerDP->addAttribute("i_gen", 	 genDP->attribute("i_intf"));
    loggerDP->addAttribute("Etorque", 	 genDP->attribute("Etorque"));
    //loggerDP->addAttribute("delta", 	 genDP->attribute("delta"));
    //loggerDP->addAttribute("w_r", 		 genDP->attribute("w_r"));
	//loggerDP->addAttribute("Edq0",		 genDP->attribute("Edq0_t"));
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

	//Simultion parameters
	Real SwitchClosed = GridParams.SwitchClosed;
	Real SwitchOpen = GridParams.SwitchOpen;
	Real startTimeFault = 1.0;
	Real endTimeFault   = 1.1;
	Real finalTime = 5;
	Real timeStep = 1e-3;
	int SGModel = 4;
	Real H = syngenKundur.H;
	std::string SGModel_str = "4Order";
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
			SGModel = args.getOptionReal("SGModel");
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
			H = args.getOptionReal("Inertia");
			inertia_str = "_Inertia_" + std::to_string(H);
		}
	}

	Real logDownSampling;
	if (timeStep<100e-6)
		logDownSampling = floor((100e-6) / timeStep);
	else
		logDownSampling = 1.0;
	Logger::Level logLevel = Logger::Level::off;
	std::string simName = "DP_SynGen" + SGModel_str + "VBR_Load_Fault" + stepSize_str + inertia_str;
	DP_1ph_SynGen_Fault(simName, timeStep, finalTime, H, startTimeFault, endTimeFault, 
						logDownSampling, SwitchOpen, SwitchClosed, SGModel, logLevel);
}