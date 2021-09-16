#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::ThreeBus;

ScenarioConfig ThreeBus;

void EMT_SynGenTrStab_3Bus_SteadyState(String simName, Real timeStep, Real finalTime, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdDamping_G1, Real cmdDamping_G2) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.initActivePower_G1, ThreeBus.setPointVoltage_G1*ThreeBus.t1_ratio, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(ThreeBus.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(ThreeBus.nomPower_G2, ThreeBus.nomPhPhVoltRMS_G2, ThreeBus.initActivePower_G2, ThreeBus.setPointVoltage_G2*ThreeBus.t2_ratio, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(ThreeBus.Vnom);

	//use Shunt as Load for powerflow
	auto loadPF = SP::Ph1::Shunt::make("Load", Logger::Level::debug);
	loadPF->setParameters(ThreeBus.activePower_L / std::pow(ThreeBus.Vnom, 2), - ThreeBus.reactivePower_L / std::pow(ThreeBus.Vnom, 2));
	loadPF->setBaseVoltage(ThreeBus.Vnom);
	
	//Line12
	auto line12PF = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12PF->setParameters(ThreeBus.lineResistance12, ThreeBus.lineInductance12, ThreeBus.lineCapacitance12, ThreeBus.lineConductance12);
	line12PF->setBaseVoltage(ThreeBus.Vnom);
	//Line13
	auto line13PF = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13PF->setParameters(ThreeBus.lineResistance13, ThreeBus.lineInductance13, ThreeBus.lineCapacitance13, ThreeBus.lineConductance13);
	line13PF->setBaseVoltage(ThreeBus.Vnom);
	//Line23
	auto line23PF = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23PF->setParameters(ThreeBus.lineResistance23, ThreeBus.lineInductance23, ThreeBus.lineCapacitance23, ThreeBus.lineConductance23);
	line23PF->setBaseVoltage(ThreeBus.Vnom);

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	loadPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, loadPF, line12PF, line13PF, line23PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameEMT = simName + "_EMT";
	Logger::setLogDir("logs/"+simNameEMT);
	
	// Nodes
	auto n1EMT = SimNode<Real>::make("n1", PhaseType::ABC);
	auto n2EMT = SimNode<Real>::make("n2", PhaseType::ABC);
	auto n3EMT = SimNode<Real>::make("n3", PhaseType::ABC);

	// Components
	//Synchronous generator 1
	auto gen1EMT = EMT::Ph3::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1EMT->setStandardParametersPU(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.nomFreq_G1, ThreeBus.Xpd_G1*std::pow(ThreeBus.t1_ratio,2), cmdInertia_G1*ThreeBus.H_G1, ThreeBus.Rs_G1, cmdDamping_G1*ThreeBus.D_G1);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= gen1PF->getApparentPower();
	gen1EMT->setInitialValues(initApparentPower_G1, ThreeBus.initMechPower_G1);

		//Synchronous generator 1
	auto gen2EMT = EMT::Ph3::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2EMT->setStandardParametersPU(ThreeBus.nomPower_G2, ThreeBus.nomPhPhVoltRMS_G2, ThreeBus.nomFreq_G2, ThreeBus.Xpd_G2*std::pow(ThreeBus.t2_ratio,2), cmdInertia_G2*ThreeBus.H_G2, ThreeBus.Rs_G2, cmdDamping_G2*ThreeBus.D_G2);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2= gen2PF->getApparentPower();
	gen2EMT->setInitialValues(initApparentPower_G2, ThreeBus.initMechPower_G2);
	
	//Load
	auto loadEMT = EMT::Ph3::RXLoad::make("Load", Logger::Level::debug);
	loadEMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(ThreeBus.activePower_L), CPS::Math::singlePhasePowerToThreePhase(ThreeBus.reactivePower_L), ThreeBus.Vnom);

	// Line12
	auto line12EMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	line12EMT->setParameters(Math::singlePhaseParameterToThreePhase(ThreeBus.lineResistance12), 
	                      Math::singlePhaseParameterToThreePhase(ThreeBus.lineInductance12), 
					      Math::singlePhaseParameterToThreePhase(ThreeBus.lineCapacitance12),
						  Math::singlePhaseParameterToThreePhase(ThreeBus.lineConductance12));

	// Line13
	auto line13EMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	line13EMT->setParameters(Math::singlePhaseParameterToThreePhase(ThreeBus.lineResistance13), 
	                      Math::singlePhaseParameterToThreePhase(ThreeBus.lineInductance13), 
					      Math::singlePhaseParameterToThreePhase(ThreeBus.lineCapacitance13),
						  Math::singlePhaseParameterToThreePhase(ThreeBus.lineConductance13));

	// Line23
	auto line23EMT = EMT::Ph3::PiLine::make("PiLine", Logger::Level::debug);
	line23EMT->setParameters(Math::singlePhaseParameterToThreePhase(ThreeBus.lineResistance23), 
	                      Math::singlePhaseParameterToThreePhase(ThreeBus.lineInductance23), 
					      Math::singlePhaseParameterToThreePhase(ThreeBus.lineCapacitance23),
						  Math::singlePhaseParameterToThreePhase(ThreeBus.lineConductance23));

// Topology
	gen1EMT->connect({ n1EMT });
	gen2EMT->connect({ n2EMT });
	loadEMT->connect({ n3EMT });
	line12EMT->connect({ n1EMT, n2EMT });
	line13EMT->connect({ n1EMT, n3EMT });
	line23EMT->connect({ n2EMT, n3EMT });
	auto systemEMT = SystemTopology(60,
			SystemNodeList{n1EMT, n2EMT, n3EMT},
			SystemComponentList{gen1EMT, gen2EMT, loadEMT, line12EMT, line13EMT, line23EMT});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameEMT, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemEMT);


	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->addAttribute("v1", n1EMT->attribute("v"));
	loggerEMT->addAttribute("v2", n2EMT->attribute("v"));
	loggerEMT->addAttribute("v3", n3EMT->attribute("v"));
	loggerEMT->addAttribute("v_line12", line12EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line12", line12EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line13", line13EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line13", line13EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line23", line23EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line23", line23EMT->attribute("i_intf"));
	loggerEMT->addAttribute("Ep_gen1", gen1EMT->attribute("Ep_mag"));
	loggerEMT->addAttribute("v_gen1", gen1EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen1", gen1EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen1", gen1EMT->attribute("w_r"));
	loggerEMT->addAttribute("delta_gen1", gen1EMT->attribute("delta_r"));
	loggerEMT->addAttribute("Ep_gen2", gen2EMT->attribute("Ep_mag"));
	loggerEMT->addAttribute("v_gen2", gen2EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen2", gen2EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen2", gen2EMT->attribute("w_r"));	
	// loggerEMT->addAttribute("wref_gen2", gen2EMT->attribute("w_ref"));
	loggerEMT->addAttribute("delta_gen2", gen2EMT->attribute("delta_r"));	
	loggerEMT->addAttribute("v_load", loadEMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_load", loadEMT->attribute("i_intf"));
	loggerEMT->addAttribute("P_mech1", gen1EMT->attribute("P_mech"));
	loggerEMT->addAttribute("P_mech2", gen2EMT->attribute("P_mech"));
	loggerEMT->addAttribute("P_elec1", gen1EMT->attribute("P_elec"));
	loggerEMT->addAttribute("P_elec2", gen2EMT->attribute("P_elec"));

	Simulation simEMT(simNameEMT, Logger::Level::debug);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);

	simEMT.run();
}

int main(int argc, char* argv[]) {	
		

	//Simultion parameters
	String simName="EMT_SynGenTrStab_3Bus_SteadyState";
	Real finalTime = 30;
	Real timeStep = 0.00005;
	Real cmdInertia_G1= 1.0;
	Real cmdInertia_G2= 1.0;
	Real cmdDamping_G1=1.0;
	Real cmdDamping_G2=1.0;

		CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA_G1") != args.options.end())
			cmdInertia_G1 = args.options["SCALEINERTIA_G1"];
		if (args.options.find("SCALEINERTIA_G2") != args.options.end())
			cmdInertia_G2 = args.options["SCALEINERTIA_G2"];
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.options["SCALEDAMPING_G1"];
		if (args.options.find("SCALEDAMPING_G2") != args.options.end())
			cmdDamping_G2 = args.options["SCALEDAMPING_G2"];
	}

	EMT_SynGenTrStab_3Bus_SteadyState(simName, timeStep, finalTime, cmdInertia_G1, cmdInertia_G2, cmdDamping_G1, cmdDamping_G2);
}