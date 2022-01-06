/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::KRK_TwoArea;
using namespace CIM::Examples;

ScenarioConfig KRK_TwoArea;

void DP_SynGenTrStab_KRK_TwoArea_Fault(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdInertia_G3, Real cmdInertia_G4, Real cmdDamping_G1, Real cmdDamping_G2, Real cmdDamping_G3, Real cmdDamping_G4) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+ timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	// auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	// auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	// auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);
    // auto n4PF = SimNode<Complex>::make("n4", PhaseType::Single);
	auto n5PF = SimNode<Complex>::make("n5", PhaseType::Single);
	auto n6PF = SimNode<Complex>::make("n6", PhaseType::Single);
    auto n7PF = SimNode<Complex>::make("n7", PhaseType::Single);
	auto n8PF = SimNode<Complex>::make("n8", PhaseType::Single);
	auto n9PF = SimNode<Complex>::make("n9", PhaseType::Single);
    auto n10PF = SimNode<Complex>::make("n10", PhaseType::Single);
	auto n11PF = SimNode<Complex>::make("n11", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator1", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.initActivePower_G1, KRK_TwoArea.setPointVoltage_G1*KRK_TwoArea.t1_ratio, PowerflowBusType::PV);
	gen1PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator2", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.initActivePower_G2, KRK_TwoArea.setPointVoltage_G2*KRK_TwoArea.t2_ratio, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 3
	auto gen3PF = SP::Ph1::SynchronGenerator::make("Generator3", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen3PF->setParameters(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.initActivePower_G3, KRK_TwoArea.setPointVoltage_G3*KRK_TwoArea.t3_ratio, PowerflowBusType::VD);
	gen3PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 4
	auto gen4PF = SP::Ph1::SynchronGenerator::make("Generator4", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen4PF->setParameters(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.initActivePower_G4, KRK_TwoArea.setPointVoltage_G4*KRK_TwoArea.t4_ratio, PowerflowBusType::PV);
	gen4PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//use Shunt as Load for powerflow
	auto load7PF = SP::Ph1::Shunt::make("Load7", Logger::Level::debug);
	load7PF->setParameters(KRK_TwoArea.activePower_L7 / std::pow(KRK_TwoArea.Vnom, 2), (-KRK_TwoArea.reactivePower_L7_inductive) / std::pow(KRK_TwoArea.Vnom, 2));
	load7PF->setBaseVoltage(KRK_TwoArea.Vnom);
	// auto load7PF = SP::Ph1::Load::make("Load7", Logger::Level::debug);	
	// load7PF->setParameters(KRK_TwoArea.activePower_L7 , (KRK_TwoArea.reactivePower_L7_inductive + KRK_TwoArea.reactivePower_L7_capacitive), KRK_TwoArea.Vnom );


    auto load9PF = SP::Ph1::Shunt::make("Load9", Logger::Level::debug);
	load9PF->setParameters(KRK_TwoArea.activePower_L9 / std::pow(KRK_TwoArea.Vnom, 2), (-KRK_TwoArea.reactivePower_L9_inductive) / std::pow(KRK_TwoArea.Vnom, 2));
	load9PF->setBaseVoltage(KRK_TwoArea.Vnom);
	// auto load9PF = SP::Ph1::Load::make("Load9", Logger::Level::debug);
	// load9PF->setParameters(KRK_TwoArea.activePower_L9 , (KRK_TwoArea.reactivePower_L9_inductive + KRK_TwoArea.reactivePower_L9_capacitive), KRK_TwoArea.Vnom);

	//Line56
	auto line56PF = SP::Ph1::PiLine::make("PiLine56", Logger::Level::debug);
	line56PF->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	line56PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line67
	auto line67PF = SP::Ph1::PiLine::make("PiLine67", Logger::Level::debug);
	line67PF->setParameters(KRK_TwoArea.lineResistance67, KRK_TwoArea.lineInductance67, KRK_TwoArea.lineCapacitance67, KRK_TwoArea.lineConductance67);
	line67PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line78_1
	auto line78_1PF = SP::Ph1::PiLine::make("Piline78_1", Logger::Level::debug);
	line78_1PF->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	line78_1PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line78_2
	auto line78_2PF = SP::Ph1::PiLine::make("Piline78_2", Logger::Level::debug);
	line78_2PF->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	line78_2PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line89_1
	auto line89_1PF = SP::Ph1::PiLine::make("Piline89_1", Logger::Level::debug);
	line89_1PF->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	line89_1PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line89_2
	auto line89_2PF = SP::Ph1::PiLine::make("Piline89_2", Logger::Level::debug);
	line89_2PF->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	line89_2PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line910
	auto line910PF = SP::Ph1::PiLine::make("PiLine910", Logger::Level::debug);
	line910PF->setParameters(KRK_TwoArea.lineResistance910, KRK_TwoArea.lineInductance910, KRK_TwoArea.lineCapacitance910, KRK_TwoArea.lineConductance910);
	line910PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line1011
	auto line1011PF = SP::Ph1::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011PF->setParameters(KRK_TwoArea.lineResistance1011, KRK_TwoArea.lineInductance1011, KRK_TwoArea.lineCapacitance1011, KRK_TwoArea.lineConductance1011);
	line1011PF->setBaseVoltage(KRK_TwoArea.Vnom);

	// Topology
	gen1PF->connect({ n5PF });
	gen2PF->connect({ n6PF });
    gen3PF->connect({ n11PF });
    gen4PF->connect({ n10PF });
	load7PF->connect({ n7PF });
    load9PF->connect({ n9PF });
	line56PF->connect({ n5PF, n6PF });
	line67PF->connect({ n6PF, n7PF });
	line78_1PF->connect({ n7PF, n8PF });
	line78_2PF->connect({ n7PF, n8PF });
    line89_1PF->connect({ n8PF, n9PF });
	line89_2PF->connect({ n8PF, n9PF });
    line910PF->connect({ n9PF, n10PF });
    line1011PF->connect({ n10PF, n11PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{ n5PF, n6PF, n7PF, n8PF, n9PF, n10PF, n11PF},
			SystemComponentList{gen1PF, gen2PF, gen3PF, gen4PF, load7PF, load9PF, line56PF, line67PF, line78_1PF, line78_2PF, line89_1PF, line89_2PF, line910PF, line1011PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	// loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	// loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	// loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));
    // loggerPF->addAttribute("v_bus4", n4PF->attribute("v"));
	loggerPF->addAttribute("v_bus5", n5PF->attribute("v"));
	loggerPF->addAttribute("v_bus6", n6PF->attribute("v"));
    loggerPF->addAttribute("v_bus7", n7PF->attribute("v"));
	loggerPF->addAttribute("v_bus8", n8PF->attribute("v"));
	loggerPF->addAttribute("v_bus9", n9PF->attribute("v"));
    loggerPF->addAttribute("v_bus10", n10PF->attribute("v"));
	loggerPF->addAttribute("v_bus11", n11PF->attribute("v"));

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
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/"+simNameDP);

	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	// auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	// auto n3DP = SimNode<Complex>::make("n3", PhaseType::Single);
	// auto n4DP = SimNode<Complex>::make("n4", PhaseType::Single);
	auto n5DP = SimNode<Complex>::make("n5", PhaseType::Single);
	auto n6DP = SimNode<Complex>::make("n6", PhaseType::Single);
	auto n7DP = SimNode<Complex>::make("n7", PhaseType::Single);
	auto n8DP = SimNode<Complex>::make("n8", PhaseType::Single);
	auto n9DP = SimNode<Complex>::make("n9", PhaseType::Single);
	auto n10DP = SimNode<Complex>::make("n10", PhaseType::Single);
	auto n11DP = SimNode<Complex>::make("n11", PhaseType::Single);

	// Dynamic simulation, operational parameters for all the generators
	// Real Rs = 0.0025;
	// Real Ld = 1.8;
	// Real Lq = 1.7;
	// Real Ld_t = 0.2999;
	// Real Lq_t = 0.5500;
	// Real Ld_s = 0.2500;
	// Real Lq_s = 0.2500;
	Real Ll = 0.2;
	// Real Td0_t = 8.0;
	// Real Tq0_t = 0.45;
	// Real Td0_s = 0.0300;
	// Real Tq0_s = 0.05;

	// Dynamic simulation, fundamental parameters for all the generators
	Real L_ad = 1.6;
	// Real L_aq = 1.5;
	Real L_fd = 0.10655;
	// Real R_fd = 0.00056;
	// Real L_1d = 0.10010;
	// Real R_1d = 0.01768;
	// Real L_1q = 0.45652;
	// Real R_1q = 0.01153;
	// Real L_2q = 0.05833;
	// Real R_2q = 0.02166; 

	// Initial information from Kundur's description
	// Real init_terminal_volt_G1 = KRK_TwoArea.setPointVoltage_G1*KRK_TwoArea.t1_ratio;
	// Real init_volt_angle_G1 = np.deg2rad(20.2);
	// Real initActivePower_G1 = 700e6;
	// Real initReactivePower_G1 = 185e6;
	// Complex initApparentPower_G1 = 700e6 + 185e6i;

	// Real init_terminal_volt_G2 = KRK_TwoArea.setPointVoltage_G2*KRK_TwoArea.t2_ratio;
	// Real init_volt_angle_G2 = np.deg2rad(10.5);
	// Real initActivePower_G2 = 700e6;
	// Real initReactivePower_G2 = 235e6;
	// Complex initApparentPower_G2 = 700e6 + 235e6i;

	// Real init_terminal_volt_G3 = KRK_TwoArea.setPointVoltage_G3*KRK_TwoArea.t3_ratio;
	// Real init_volt_angle_G3 = np.deg2rad(-6.8);
	// Real initActivePower_G3 = 719e6;
	// Real initReactivePower_G3 = 176e6;
	// Complex initApparentPower_G3 = 719e6 + 176e6i;

	// Real init_terminal_volt_G4 = KRK_TwoArea.setPointVoltage_G4*KRK_TwoArea.t4_ratio;
	// Real init_volt_angle_G4 = np.deg2rad(-17.0);
	// Real initActivePower_G4 = 700e6;
	// Real initReactivePower_G4 = 202e6;
	// Complex initApparentPower_G4 = 700e6 + 202e6i;

	// Components
	//Synchronous generator 1
	auto gen1DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen1", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	// gen1DP->setOperationalPerUnitParameters(2, KRK_TwoArea.H_G1, Rs, Ld, Lq, Ll, Ld_t, Lq_t, Ld_s, Lq_s, Td0_t, Tq0_t, Td0_s, Tq0_s);
	gen1DP->setFundamentalParametersPU(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.nomFreq_G1, Ll, L_ad, L_fd, KRK_TwoArea.H_G1);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1 = gen1PF->getApparentPower();
	gen1DP->setInitialValues(initApparentPower_G1, KRK_TwoArea.initMechPower_G1);

	// auto gen1DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// // Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	// gen1DP->setStandardParametersPU(KRK_TwoArea.nomPower_G1_alt, KRK_TwoArea.nomPhPhVoltRMS_G1_alt, KRK_TwoArea.nomFreq_G1_alt, KRK_TwoArea.Xpd_G1*std::pow(KRK_TwoArea.t1_ratio_alt,2), cmdInertia_G1*KRK_TwoArea.H_G1, KRK_TwoArea.Rs_G1, cmdDamping_G1*KRK_TwoArea.D_G1);
	// // Get actual active and reactive power of generator's Terminal from Powerflow solution
	// Complex initApparentPower_G1= gen1PF->getApparentPower();
	// gen1DP->setInitialValues(initApparentPower_G1, KRK_TwoArea.initMechPower_G1_alt);

	//Synchronous generator 2
	auto gen2DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen2", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2DP->setFundamentalParametersPU(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.nomFreq_G2, Ll, L_ad, L_fd, KRK_TwoArea.H_G2);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2 = gen2PF->getApparentPower();
	gen2DP->setInitialValues(initApparentPower_G2, KRK_TwoArea.initMechPower_G2);

	//Synchronous generator 3
	auto gen3DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen3", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen3DP->setFundamentalParametersPU(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.nomFreq_G3, Ll, L_ad, L_fd, KRK_TwoArea.H_G3);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G3 = gen3PF->getApparentPower();
	gen3DP->setInitialValues(initApparentPower_G3, KRK_TwoArea.initMechPower_G3);

	//Synchronous generator 4
	auto gen4DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen4", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen4DP->setFundamentalParametersPU(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.nomFreq_G4, Ll, L_ad, L_fd, KRK_TwoArea.H_G4);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G4 = gen4PF->getApparentPower();
	gen4DP->setInitialValues(initApparentPower_G4, KRK_TwoArea.initMechPower_G4);

	// gen2DP->setModelFlags(true, true);
	// gen2DP->setReferenceOmega(gen1DP->attribute<Real>("w_r"), gen1DP->attribute<Real>("delta_r"));

	auto trafo15 = DP::Ph1::Transformer::make("trafo", "trafo", Logger::Level::debug, false);
	trafo15->connect({ n1DP, n5DP });
	trafo15->setParameters(KRK_TwoArea.Vnom, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.t1_ratio, 0, 0, KRK_TwoArea.transformerInductance);

	///Loads
	auto load7DP = DP::Ph1::RXLoad::make("Load7", Logger::Level::debug);
	load7DP->setParameters(KRK_TwoArea.activePower_L7, KRK_TwoArea.reactivePower_L7_inductive, KRK_TwoArea.Vnom);

	auto load9DP = DP::Ph1::RXLoad::make("Load9", Logger::Level::debug);
	load9DP->setParameters(KRK_TwoArea.activePower_L9, KRK_TwoArea.reactivePower_L9_inductive, KRK_TwoArea.Vnom);

	//Line45
	auto line45DP = DP::Ph1::PiLine::make("PiLine45", Logger::Level::debug);
	line45DP->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	//Line56
	auto line56DP = DP::Ph1::PiLine::make("PiLine56", Logger::Level::debug);
	line56DP->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	//Line67
	auto line67DP = DP::Ph1::PiLine::make("PiLine67", Logger::Level::debug);
	line67DP->setParameters(KRK_TwoArea.lineResistance67, KRK_TwoArea.lineInductance67, KRK_TwoArea.lineCapacitance67, KRK_TwoArea.lineConductance67);
	//Line78_1
	auto line78_1DP = DP::Ph1::PiLine::make("PiLine78_1", Logger::Level::debug);
	line78_1DP->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	//Line78_2
	auto line78_2DP = DP::Ph1::PiLine::make("PiLine78_2", Logger::Level::debug);
	line78_2DP->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	//Line89_1
	auto line89_1DP = DP::Ph1::PiLine::make("PiLine89_1", Logger::Level::debug);
	line89_1DP->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	//Line89_2
	auto line89_2DP = DP::Ph1::PiLine::make("PiLine89_2", Logger::Level::debug);
	line89_2DP->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	//Line910
	auto line910DP = DP::Ph1::PiLine::make("PiLine910", Logger::Level::debug);
	line910DP->setParameters(KRK_TwoArea.lineResistance910, KRK_TwoArea.lineInductance910, KRK_TwoArea.lineCapacitance910, KRK_TwoArea.lineConductance910);
	//Line1011
	auto line1011DP = DP::Ph1::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011DP->setParameters(KRK_TwoArea.lineResistance1011, KRK_TwoArea.lineInductance1011, KRK_TwoArea.lineCapacitance1011, KRK_TwoArea.lineConductance1011);

	// // //Switch
	// // auto faultDP = DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	// // faultDP->setParameters(SwitchOpen, SwitchClosed);
	// // faultDP->open();

	// //Variable resistance Switch
	// auto faultDP = DP::Ph1::varResSwitch::make("Br_fault", Logger::Level::debug);
	// faultDP->setParameters(SwitchOpen, SwitchClosed);
	// faultDP->setInitParameters(timeStep);
	// faultDP->open();

	// Topology
	gen1DP->connect({ n1DP });
	gen2DP->connect({ n6DP });
	gen3DP->connect({ n11DP });
	gen4DP->connect({ n10DP });
	load7DP->connect({ n7DP });
	load9DP->connect({ n9DP });
	line56DP->connect({ n5DP, n6DP });
	line67DP->connect({ n6DP, n7DP });
	line78_1DP->connect({ n7DP, n8DP });
	line78_2DP->connect({ n7DP, n8DP });
	line89_1DP->connect({ n8DP, n9DP });
	line89_2DP->connect({ n8DP, n9DP });
	line910DP->connect({ n9DP, n10DP });
	line1011DP->connect({ n10DP, n11DP });
	// faultDP->connect({DP::SimNode::GND , n1DP }); //terminal of generator 1
	// faultDP->connect({DP::SimNode::GND , n2DP }); //terminal of generator 2
	// faultDP->connect({DP::SimNode::GND , n3DP }); //Load bus
	auto systemDP = SystemTopology(60,
			SystemNodeList{n5DP, n6DP, n7DP, n8DP, n9DP, n10DP, n11DP},
			SystemComponentList{gen1DP, gen2DP, gen3DP, gen4DP, load7DP, load9DP, trafo15, line56DP, line67DP, line78_1DP, line78_2DP, line89_1DP, line89_2DP, line910DP, line1011DP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v5", n5DP->attribute("v"));
	loggerDP->addAttribute("v6", n6DP->attribute("v"));
	loggerDP->addAttribute("v7", n7DP->attribute("v"));
	loggerDP->addAttribute("v8", n8DP->attribute("v"));
	loggerDP->addAttribute("v9", n9DP->attribute("v"));
	loggerDP->addAttribute("v10", n10DP->attribute("v"));
	loggerDP->addAttribute("v11", n11DP->attribute("v"));
	loggerDP->addAttribute("v_line56", line56DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line56", line56DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line67", line67DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line67", line67DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line78_1", line78_1DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line78_1", line78_1DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line78_2", line78_2DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line78_2", line78_2DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line89_1", line89_1DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line89_1", line89_1DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line89_2", line89_2DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line89_2", line89_2DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line910", line910DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line910", line910DP->attribute("i_intf"));
	loggerDP->addAttribute("v_line1011", line1011DP->attribute("v_intf"));
	loggerDP->addAttribute("i_line1011", line1011DP->attribute("i_intf"));
	loggerDP->addAttribute("Ep_gen1", gen1DP->attribute("Ep_mag"));
	loggerDP->addAttribute("v_gen1", gen1DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen1", gen1DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen1", gen1DP->attribute("w_r"));
	loggerDP->addAttribute("delta_gen1", gen1DP->attribute("delta_r"));
	loggerDP->addAttribute("Ep_gen2", gen2DP->attribute("Ep_mag"));
	loggerDP->addAttribute("v_gen2", gen2DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen2", gen2DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen2", gen2DP->attribute("w_r"));
	// loggerDP->addAttribute("wref_gen2", gen2DP->attribute("w_ref"));
	loggerDP->addAttribute("delta_gen2", gen2DP->attribute("delta_r"));
	loggerDP->addAttribute("Ep_gen3", gen3DP->attribute("Ep_mag"));
	loggerDP->addAttribute("v_gen3", gen3DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen3", gen3DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen3", gen3DP->attribute("w_r"));
	loggerDP->addAttribute("delta_gen3", gen3DP->attribute("delta_r"));
	loggerDP->addAttribute("Ep_gen4", gen4DP->attribute("Ep_mag"));
	loggerDP->addAttribute("v_gen4", gen4DP->attribute("v_intf"));
	loggerDP->addAttribute("i_gen4", gen4DP->attribute("i_intf"));
	loggerDP->addAttribute("wr_gen4", gen4DP->attribute("w_r"));
	loggerDP->addAttribute("delta_gen4", gen4DP->attribute("delta_r"));
	// loggerDP->addAttribute("i_fault", faultDP->attribute("i_intf"));
	loggerDP->addAttribute("v_load7", load7DP->attribute("v_intf"));
	loggerDP->addAttribute("i_load7", load7DP->attribute("i_intf"));
	loggerDP->addAttribute("v_load9", load9DP->attribute("v_intf"));
	loggerDP->addAttribute("i_load9", load9DP->attribute("i_intf"));
	loggerDP->addAttribute("P_mech1", gen1DP->attribute("P_mech"));
	loggerDP->addAttribute("P_mech2", gen2DP->attribute("P_mech"));
	loggerDP->addAttribute("P_elec1", gen1DP->attribute("P_elec"));
	loggerDP->addAttribute("P_elec2", gen2DP->attribute("P_elec"));
	loggerDP->addAttribute("P_mech3", gen3DP->attribute("P_mech"));
	loggerDP->addAttribute("P_mech4", gen4DP->attribute("P_mech"));
	loggerDP->addAttribute("P_elec3", gen3DP->attribute("P_elec"));
	loggerDP->addAttribute("P_elec4", gen4DP->attribute("P_elec"));

	Simulation simDP(simNameDP, Logger::Level::debug);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.addLogger(loggerDP);
	simDP.doSystemMatrixRecomputation(true);

	// // Events
	// if (startFaultEvent){
	// 	auto sw1 = SwitchEvent::make(startTimeFault, faultDP, true);

	// 	simDP.addEvent(sw1);
	// }

	// if(endFaultEvent){

	// 	auto sw2 = SwitchEvent::make(endTimeFault, faultDP, false);
	// 	simDP.addEvent(sw2);

	// }

	simDP.run();
}

int main(int argc, char* argv[]) {


	//Simulation parameters
	String simName="DP_SynGenTrStab_KRK_TwoArea_Fault";
	Real finalTime = 30;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.2;
	Real cmdInertia_G1 = KRK_TwoArea.H_G1;
	Real cmdInertia_G2 = KRK_TwoArea.H_G2;
    Real cmdInertia_G3 = KRK_TwoArea.H_G3;
    Real cmdInertia_G4 = KRK_TwoArea.H_G4;
	Real cmdDamping_G1=1.0;
	Real cmdDamping_G2=1.0;
    Real cmdDamping_G3=1.0;
    Real cmdDamping_G4=1.0;

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
        if (args.options.find("SCALEINERTIA_G3") != args.options.end())
			cmdInertia_G3 = args.options["SCALEINERTIA_G3"];
        if (args.options.find("SCALEINERTIA_G4") != args.options.end())
			cmdInertia_G4 = args.options["SCALEINERTIA_G4"];
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.options["SCALEDAMPING_G1"];
		if (args.options.find("SCALEDAMPING_G2") != args.options.end())
			cmdDamping_G2 = args.options["SCALEDAMPING_G2"];
        if (args.options.find("SCALEDAMPING_G3") != args.options.end())
			cmdDamping_G3 = args.options["SCALEDAMPING_G3"];
        if (args.options.find("SCALEDAMPING_G4") != args.options.end())
			cmdDamping_G4 = args.options["SCALEDAMPING_G4"];
		if (args.options.find("STARTTIMEFAULT") != args.options.end())
			startTimeFault = args.options["STARTTIMEFAULT"];
		if (args.options.find("ENDTIMEFAULT") != args.options.end())
			endTimeFault = args.options["ENDTIMEFAULT"];
	}

	DP_SynGenTrStab_KRK_TwoArea_Fault(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia_G1, cmdInertia_G2, cmdInertia_G3, cmdInertia_G4, cmdDamping_G1, cmdDamping_G2, cmdDamping_G3, cmdDamping_G4);
}
