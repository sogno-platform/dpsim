#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::Three_bus_cfg;

ScenarioConfig Three_bus_cfg;

void Three_bus_sim(String simName, Real timeStep, Real finalTime, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdDamping_G1, Real cmdDamping_G2)
{
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime + timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	// nodes
	auto n1_PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2_PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3_PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//***Applying the new signal model: PMUSignalDevice
	//***Tonstruction a PMU
	auto pmu_1 = Signal::PMUSignalDevice::make("PMU_1", Logger::Level::debug);
	// //***The attribute mInput of PMU is set by the mVoltage of the SimNode
	 pmu_1->mInput->setReference(n1_PF->mVoltage);

	auto pmu_2 = Signal::PMUSignalDevice::make("PMU_2", Logger::Level::debug);
	pmu_2->mInput->setReference(n2_PF->mVoltage);

	auto pmu_3 = Signal::PMUSignalDevice::make("PMU_3", Logger::Level::debug);
	pmu_3->mInput->setReference(n3_PF->mVoltage);

	// injection
	// configuration(Synchronous generator 1)
	auto gen1_PF = SP::Ph1::SynchronGenerator::make("Generator_1", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1_PF->setParameters(Three_bus_cfg.nomPower_G1, Three_bus_cfg.nomPhPhVoltRMS_G1, Three_bus_cfg.initActivePower_G1, Three_bus_cfg.setPointVoltage_G1 * Three_bus_cfg.t1_ratio, PowerflowBusType::VD, Three_bus_cfg.initReactivePower_G1);
	gen1_PF->setBaseVoltage(Three_bus_cfg.Vnom);

	// Synchronous generator 2
	auto gen2_PF = SP::Ph1::SynchronGenerator::make("Generator_2", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2_PF->setParameters(Three_bus_cfg.nomPower_G2, Three_bus_cfg.nomPhPhVoltRMS_G2, Three_bus_cfg.initActivePower_G2, Three_bus_cfg.setPointVoltage_G2 * Three_bus_cfg.t2_ratio, PowerflowBusType::PV, Three_bus_cfg.initReactivePower_G2);
	gen2_PF->setBaseVoltage(Three_bus_cfg.Vnom);

	// use Shunt as Load for powerflow
	auto load_PF = SP::Ph1::Shunt::make("Load", Logger::Level::debug);
	load_PF->setParameters(Three_bus_cfg.activePower_L / std::pow(Three_bus_cfg.Vnom, 2), -Three_bus_cfg.reactivePower_L / std::pow(Three_bus_cfg.Vnom, 2));
	load_PF->setBaseVoltage(Three_bus_cfg.Vnom);

	// Line12
	auto line_pf_1 = SP::Ph1::PiLine::make("PiLine_13", Logger::Level::debug);
	line_pf_1->setParameters(Three_bus_cfg.lineResistance, Three_bus_cfg.lineInductance, Three_bus_cfg.lineCapacitance);
	line_pf_1->setBaseVoltage(Three_bus_cfg.Vnom);
	// Line13
	auto line_pf_2 = SP::Ph1::PiLine::make("PiLine_31", Logger::Level::debug);
	line_pf_2->setParameters(Three_bus_cfg.lineResistance, Three_bus_cfg.lineInductance, Three_bus_cfg.lineCapacitance);
	line_pf_2->setBaseVoltage(Three_bus_cfg.Vnom);
	// Line23
	auto line_pf_3 = SP::Ph1::PiLine::make("PiLine_23", Logger::Level::debug);
	line_pf_3->setParameters(Three_bus_cfg.lineResistance, 2 * Three_bus_cfg.lineInductance, 2 * Three_bus_cfg.lineCapacitance);
	line_pf_3->setBaseVoltage(Three_bus_cfg.Vnom);

	// Topology
	gen1_PF->connect({n1_PF});
	gen2_PF->connect({n2_PF});
	load_PF->connect({n3_PF});
	line_pf_1->connect({n1_PF, n2_PF});
	line_pf_2->connect({n1_PF, n3_PF});
	line_pf_3->connect({n2_PF, n3_PF});

	auto systemPF = SystemTopology(50,
								   SystemNodeList{n1_PF, n2_PF, n3_PF},
								   SystemComponentList{gen1_PF, gen2_PF, line_pf_1, line_pf_2, line_pf_3, load_PF});

	//***Adding the PMU into the system topology
	 systemPF.mComponents.push_back(pmu_1);
	 systemPF.mComponents.push_back(pmu_2);
	 systemPF.mComponents.push_back(pmu_3);

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v_bus1", n1_PF->attribute("v"));
	loggerPF->logAttribute("v_bus2", n2_PF->attribute("v"));
	loggerPF->logAttribute("v_bus3", n3_PF->attribute("v"));

	//***Logging the data from PMU
	loggerPF->logAttribute("pmu_input_1", pmu_1->attribute("input"));
	loggerPF->logAttribute("pmu_output_1", pmu_1->attribute("output"));

	loggerPF->logAttribute("pmu_input_2", pmu_2->attribute("input"));
	loggerPF->logAttribute("pmu_output_2", pmu_2->attribute("output"));
	loggerPF->logAttribute("pmu_input_3", pmu_3->attribute("input"));
	loggerPF->logAttribute("pmu_output_3", pmu_3->attribute("output"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);

	simPF.run();
}

int main(int argc, char *argv[])
{

	// Simultion parameters
	String simName = "Three_bus_sim";
	Real finalTime = 50;
	Real timeStep = 0.001;
	Real cmdInertia_G1 = 1.0;
	Real cmdInertia_G2 = 1.0;
	Real cmdDamping_G1 = 1.0;
	Real cmdDamping_G2 = 1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1)
	{
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA_G1") != args.options.end())
			cmdInertia_G1 = args.getOptionReal("SCALEINERTIA_G1");
		if (args.options.find("SCALEINERTIA_G2") != args.options.end())
			cmdInertia_G2 = args.getOptionReal("SCALEINERTIA_G2");
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.getOptionReal("SCALEDAMPING_G1");
		if (args.options.find("SCALEDAMPING_G2") != args.options.end())
			cmdDamping_G2 = args.getOptionReal("SCALEDAMPING_G2");
	}

	Three_bus_sim(simName, timeStep, finalTime, cmdInertia_G1, cmdInertia_G2, cmdDamping_G1, cmdDamping_G2);
}