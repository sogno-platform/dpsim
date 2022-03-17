/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University; Universidad
 *                     Nacional de Colombia
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

void VBR_SynGenDQ7odTrapez_KRK_TwoArea_SteadyState(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdInertia_G3, Real cmdInertia_G4, Real cmdDamping_G1, Real cmdDamping_G2, Real cmdDamping_G3, Real cmdDamping_G4) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+ timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
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
	gen1PF->setParameters(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.initActivePower_G1, KRK_TwoArea.setPointVoltage_G1*KRK_TwoArea.t1_ratio, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G1);
	gen1PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator2", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.initActivePower_G2, KRK_TwoArea.setPointVoltage_G2*KRK_TwoArea.t2_ratio, PowerflowBusType::VD, KRK_TwoArea.initReactivePower_G2);
	gen2PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 3
	auto gen3PF = SP::Ph1::SynchronGenerator::make("Generator3", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen3PF->setParameters(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.initActivePower_G3, KRK_TwoArea.setPointVoltage_G3*KRK_TwoArea.t3_ratio, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G3);
	gen3PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 4
	auto gen4PF = SP::Ph1::SynchronGenerator::make("Generator4", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen4PF->setParameters(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.initActivePower_G4, KRK_TwoArea.setPointVoltage_G4*KRK_TwoArea.t4_ratio, PowerflowBusType::PV, KRK_TwoArea.initReactivePower_G4);
	gen4PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //use Shunt as Load for powerflow
	// auto load7_1PF = SP::Ph1::Load::make("Load7_1", Logger::Level::debug);
	// load7_1PF->setParameters(KRK_TwoArea.activePower_L7_1, KRK_TwoArea.reactivePower_L7_1, KRK_TwoArea.Vnom);
	// auto load7_2PF = SP::Ph1::Load::make("Load7_2", Logger::Level::debug);
	// load7_2PF->setParameters(0, -KRK_TwoArea.reactivePower_L7_2, KRK_TwoArea.Vnom);

	// auto load9_1PF = SP::Ph1::Load::make("Load9_1", Logger::Level::debug);
	// load9_1PF->setParameters(KRK_TwoArea.activePower_L9_1, KRK_TwoArea.reactivePower_L9_1, KRK_TwoArea.Vnom);
	// auto load9_2PF = SP::Ph1::Load::make("Load9_2", Logger::Level::debug);
	// load9_2PF->setParameters(0, -KRK_TwoArea.reactivePower_L9_2, KRK_TwoArea.Vnom);

	auto load7PF = SP::Ph1::Load::make("Load7", Logger::Level::debug);
	load7PF->setParameters(KRK_TwoArea.activePower_L7, KRK_TwoArea.reactivePower_L7_inductive - KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7PF_c = SP::Ph1::Load::make("Load7_c", Logger::Level::debug);
	// load7PF_c->setParameters(0, -KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7PF_c = SP::Ph1::Shunt::make("Load7_c", Logger::Level::debug);
	// load7PF_c->setParameters(0, KRK_TwoArea.reactivePower_L7_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load7PF_c->setBaseVoltage(KRK_TwoArea.Vnom);

	auto load9PF = SP::Ph1::Load::make("Load9", Logger::Level::debug);
	load9PF->setParameters(KRK_TwoArea.activePower_L9, KRK_TwoArea.reactivePower_L9_inductive - KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9PF_c = SP::Ph1::Load::make("Load9_c", Logger::Level::debug);
	// load9PF_c->setParameters(0, -KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9PF_c = SP::Ph1::Shunt::make("Load9_c", Logger::Level::debug);
	// load9PF_c->setParameters(0, KRK_TwoArea.reactivePower_L9_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load9PF_c->setBaseVoltage(KRK_TwoArea.Vnom);

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

	// load7_1PF->connect({ n7PF });
    // load9_1PF->connect({ n9PF });
	// load7_2PF->connect({ n7PF });
    // load9_2PF->connect({ n9PF });

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
	loggerPF->addAttribute("v_bus5", n5PF->attribute("v"));
	loggerPF->addAttribute("s_bus5", n5PF->attribute("s"));
	loggerPF->addAttribute("v_bus6", n6PF->attribute("v"));
	loggerPF->addAttribute("s_bus6", n6PF->attribute("s"));
    loggerPF->addAttribute("v_bus7", n7PF->attribute("v"));
	loggerPF->addAttribute("s_bus7", n7PF->attribute("s"));
	loggerPF->addAttribute("v_bus8", n8PF->attribute("v"));
	loggerPF->addAttribute("s_bus8", n8PF->attribute("s"));
	loggerPF->addAttribute("v_bus9", n9PF->attribute("v"));
	loggerPF->addAttribute("s_bus9", n9PF->attribute("s"));
    loggerPF->addAttribute("v_bus10", n10PF->attribute("v"));
	loggerPF->addAttribute("s_bus10", n10PF->attribute("s"));
	loggerPF->addAttribute("v_bus11", n11PF->attribute("v"));
	loggerPF->addAttribute("s_bus11", n11PF->attribute("s"));

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

	// Generator's operational parameters
	Real Rs = 0.0025;
    Real Ld = 1.8;
    Real Lq = 1.7;
    Real Ld_t = 0.3;
    Real Lq_t = 0.55;
    Real Ld_s = 0.25;
    Real Lq_s = 0.25;
    Real Ll = 0.2;
    Real Td0_t = 8.0;
    Real Tq0_t = 0.4;
    Real Td0_s = 0.03;
    Real Tq0_s = 0.05;

	// Dynamic simulation, fundamental parameters for all the generators
    // Real L_ad = 1.6;
    // Real L_aq = 1.5;
    // Real L_fd = 0.10655;
    // Real R_fd = 0.00056;
    // Real L_1d = 0.10010;
    // Real R_1d = 0.01768;
    // Real L_1q = 0.45652;
    // Real R_1q = 0.01153;
    // Real L_2q = 0.05833;
    // Real R_2q = 0.02166;

	// Nodes
	auto n5EMT = SimNode<Real>::make("n5", PhaseType::ABC);
	auto n6EMT = SimNode<Real>::make("n6", PhaseType::ABC);
	auto n7EMT = SimNode<Real>::make("n7", PhaseType::ABC);
	auto n8EMT = SimNode<Real>::make("n8", PhaseType::ABC);
	auto n9EMT = SimNode<Real>::make("n9", PhaseType::ABC);
	auto n10EMT = SimNode<Real>::make("n10", PhaseType::ABC);
	auto n11EMT = SimNode<Real>::make("n11", PhaseType::ABC);

	// Components
	//Synchronous generator 1
	auto gen1EMT = EMT::Ph3::SynchronGeneratorVBR::make("Syngen1", Logger::Level::debug);
	gen1EMT->setBaseAndOperationalPerUnitParameters(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.nomFreq_G1, KRK_TwoArea.poleNum, KRK_TwoArea.nomFieldCurr,
		Rs, Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, Td0_t, Tq0_t, Td0_s, Tq0_s, KRK_TwoArea.H_G1);
	// Extract relevant powerflow results
	Real initTerminalVolt_G1 = std::abs(n5PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle_G1 = Math::phase(n5PF->singleVoltage()); // angle in rad
	Real initActivePower_G1 = gen1PF->getApparentPower().real();
	Real initReactivePower_G1 = gen1PF->getApparentPower().imag();
	Real initMechPower_G1 = initActivePower_G1;
	gen1EMT->setInitialValues(initActivePower_G1, initReactivePower_G1, initTerminalVolt_G1, initVoltAngle_G1, initMechPower_G1);

	auto gen2EMT = EMT::Ph3::SynchronGeneratorVBR::make("Syngen2", Logger::Level::debug);
	gen2EMT->setBaseAndOperationalPerUnitParameters(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.nomFreq_G2, KRK_TwoArea.poleNum, KRK_TwoArea.nomFieldCurr,
		Rs, Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, Td0_t, Tq0_t, Td0_s, Tq0_s, KRK_TwoArea.H_G2);
	// Extract relevant powerflow results
	Real initTerminalVolt_G2 = std::abs(n6PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle_G2 = Math::phase(n6PF->singleVoltage()); // angle in rad
	Real initActivePower_G2 = gen2PF->getApparentPower().real();
	Real initReactivePower_G2 = gen2PF->getApparentPower().imag();
	Real initMechPower_G2 = initActivePower_G2;
	gen2EMT->setInitialValues(initActivePower_G2, initReactivePower_G2, initTerminalVolt_G2, initVoltAngle_G2, initMechPower_G2);

	auto gen3EMT = EMT::Ph3::SynchronGeneratorVBR::make("Syngen3", Logger::Level::debug);
	gen3EMT->setBaseAndOperationalPerUnitParameters(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.nomFreq_G3, KRK_TwoArea.poleNum, KRK_TwoArea.nomFieldCurr,
		Rs, Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, Td0_t, Tq0_t, Td0_s, Tq0_s, KRK_TwoArea.H_G3);
	// Extract relevant powerflow results
	Real initTerminalVolt_G3 = std::abs(n11PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle_G3 = Math::phase(n11PF->singleVoltage()); // angle in rad
	Real initActivePower_G3 = gen3PF->getApparentPower().real();
	Real initReactivePower_G3 = gen3PF->getApparentPower().imag();
	Real initMechPower_G3 = initActivePower_G3;
	gen3EMT->setInitialValues(initActivePower_G3, initReactivePower_G3, initTerminalVolt_G3, initVoltAngle_G3, initMechPower_G3);

	auto gen4EMT = EMT::Ph3::SynchronGeneratorVBR::make("Syngen4", Logger::Level::debug);
	gen4EMT->setBaseAndOperationalPerUnitParameters(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.nomFreq_G4, KRK_TwoArea.poleNum, KRK_TwoArea.nomFieldCurr,
		Rs, Ld, Lq, Ld_t, Lq_t, Ld_s, Lq_s, Ll, Td0_t, Tq0_t, Td0_s, Tq0_s, KRK_TwoArea.H_G4);
	// Extract relevant powerflow results
	Real initTerminalVolt_G4 = std::abs(n10PF->singleVoltage())*RMS3PH_TO_PEAK1PH;
	Real initVoltAngle_G4 = Math::phase(n10PF->singleVoltage()); // angle in rad
	Real initActivePower_G4 = gen4PF->getApparentPower().real();
	Real initReactivePower_G4 = gen4PF->getApparentPower().imag();
	Real initMechPower_G4 = initActivePower_G4;
	gen4EMT->setInitialValues(initActivePower_G4, initReactivePower_G4, initTerminalVolt_G4, initVoltAngle_G4, initMechPower_G4);

	// gen1EMT->setModelFlags(true, true);
	// gen1EMT->setReferenceOmega(gen3EMT->attribute<Real>("w_r"), gen3EMT->attribute<Real>("delta_r"));

	// gen2EMT->setModelFlags(true, true);
	// gen2EMT->setReferenceOmega(gen3EMT->attribute<Real>("w_r"), gen3EMT->attribute<Real>("delta_r"));

	// gen4EMT->setModelFlags(true, true);
	// gen4EMT->setReferenceOmega(gen3EMT->attribute<Real>("w_r"), gen3EMT->attribute<Real>("delta_r"));

	// Exciter
	// Parameters from Kundur
	// Real Ka = 20;
	// Real Ta = 0.055;
	// Real K_E1 = -0.023286779514714816;
	// Real K_E2 = -0.023608729298954946;
	// Real K_E3 = -0.023466840265842656;
	// Real K_E4 = -0.023734231712552113;
	// Real Te = 0.36;
	// Real Kf = 0.125;
	// Real Tf = 1.8;
	// Real Tr = 0.05;

	// Parameters from Matlab
	// Real Ka = 200;
	// Real Ta = 0.001;
	// Real K_E1 = 1;
	// Real K_E2 = 1;
	// Real K_E3 = 1;
	// Real K_E4 = 1;
	// Real Te = 0;
	// Real Kf = 0;
	// Real Tf = 0;
	// Real Tr = 20e-3;

	// Governor
	// Turbine model parameters (tandem compound single reheat steam turbine, fossil-fuelled)
    // from P. Kundur, "Power System Stability and Control", 1994, p. 427
    // Real Ta_t = 0.3;    // T_CH
    // Real Fa = 0.3;      // F_HP
    // Real Tb = 7.0;      // T_RH
    // Real Fb = 0.3;      // F_IP
    // Real Tc = 0.5;      // T_CO
    // Real Fc = 0.4;      // F_LP

	// // Governor parameters (mechanical-hydraulic control)
    // // from P. Kundur, "Power System Stability and Control", 1994, p. 437
    // Real Kg = 20;       // 5% droop
    // Real Tsr = 0.001;
    // Real Tsm = 0.15;

	// gen1EMT->addExciter(Ta, Ka, Te, K_E1, Tf, Kf, Tr);
	// gen2EMT->addExciter(Ta, Ka, Te, K_E2, Tf, Kf, Tr);
	// gen3EMT->addExciter(Ta, Ka, Te, K_E3, Tf, Kf, Tr);
	// gen4EMT->addExciter(Ta, Ka, Te, K_E4, Tf, Kf, Tr);

	// gen1EMT->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initMechPower_G1 / KRK_TwoArea.nomPower_G1, initMechPower_G1 / KRK_TwoArea.nomPower_G1);
	// gen2EMT->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initMechPower_G2 / KRK_TwoArea.nomPower_G2, initMechPower_G2 / KRK_TwoArea.nomPower_G2);
	// gen3EMT->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initMechPower_G3 / KRK_TwoArea.nomPower_G3, initMechPower_G3 / KRK_TwoArea.nomPower_G3);
	// gen4EMT->addGovernor(Ta_t, Tb, Tc, Fa, Fb, Fc, Kg, Tsr, Tsm, initMechPower_G4 / KRK_TwoArea.nomPower_G4, initMechPower_G4 / KRK_TwoArea.nomPower_G4);
	
	//Loads
	// auto load7_1EMT = EMT::Ph3::RXLoad::make("Load7_1", Logger::Level::debug);
	// load7_1EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.activePower_L7_1), CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.reactivePower_L7_1), KRK_TwoArea.Vnom);
	// auto load7_2EMT = EMT::Ph3::RXLoad::make("Load7_2", Logger::Level::debug);
	// load7_2EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(0), CPS::Math::singlePhasePowerToThreePhase(-KRK_TwoArea.reactivePower_L7_2), KRK_TwoArea.Vnom);

	// auto load9_1EMT = EMT::Ph3::RXLoad::make("Load9_1", Logger::Level::debug);
	// load9_1EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.activePower_L9_1), CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.reactivePower_L9_1), KRK_TwoArea.Vnom);
	// auto load9_2EMT = EMT::Ph3::RXLoad::make("Load9_2", Logger::Level::debug);
	// load9_2EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(0), CPS::Math::singlePhasePowerToThreePhase(-KRK_TwoArea.reactivePower_L9_2), KRK_TwoArea.Vnom);

	//Loads
	auto load7EMT = EMT::Ph3::RXLoad::make("Load7", Logger::Level::debug);
	load7EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.activePower_L7), CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.reactivePower_L7_inductive - KRK_TwoArea.reactivePower_L7_capacitive), KRK_TwoArea.Vnom);
	// auto load7EMT_c = EMT::Ph1::Load::make("Load7_c", Logger::Level::debug);
	// load7EMT_c->setParameters(0, -KRK_TwoArea.reactivePower_L7_capacitive, KRK_TwoArea.Vnom);
	// auto load7EMT_c = EMT::Ph1::Shunt::make("Load7_c", Logger::Level::debug);
	// load7EMT_c->setParameters(0, KRK_TwoArea.reactivePower_L7_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load7EMT_c->setBaseVoltage(KRK_TwoArea.Vnom);

	auto load9EMT = EMT::Ph3::RXLoad::make("Load9", Logger::Level::debug);
	load9EMT->setParameters(CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.activePower_L9), CPS::Math::singlePhasePowerToThreePhase(KRK_TwoArea.reactivePower_L9_inductive - KRK_TwoArea.reactivePower_L9_capacitive), KRK_TwoArea.Vnom);
	// auto load9EMT_c = EMT::Ph1::Load::make("Load9_c", Logger::Level::debug);
	// load9EMT_c->setParameters(0, -KRK_TwoArea.reactivePower_L9_capacitive, KRK_TwoArea.Vnom);
	// auto load9EMT_c = EMT::Ph1::Shunt::make("Load9_c", Logger::Level::debug);
	// load9EMT_c->setParameters(0, KRK_TwoArea.reactivePower_L9_capacitive / std::pow(KRK_TwoArea.Vnom, 2));
	// load9EMT_c->setBaseVoltage(KRK_TwoArea.Vnom);

	//Line56
	auto line56EMT = EMT::Ph3::PiLine::make("PiLine56", Logger::Level::debug);
	line56EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance56),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance56),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance56),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance56));
	//Line67
	auto line67EMT = EMT::Ph3::PiLine::make("PiLine67", Logger::Level::debug);
	line67EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance67),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance67),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance67),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance67));
	//Line78_1
	auto line78_1EMT = EMT::Ph3::PiLine::make("PiLine78_1", Logger::Level::debug);
	line78_1EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance78),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance78),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance78),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance78));
	//Line78_2
	auto line78_2EMT = EMT::Ph3::PiLine::make("PiLine78_2", Logger::Level::debug);
	line78_2EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance78),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance78),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance78),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance78));
	//Line89_1
	auto line89_1EMT = EMT::Ph3::PiLine::make("PiLine89_1", Logger::Level::debug);
	line89_1EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance89),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance89),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance89),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance89));
	//Line89_2
	auto line89_2EMT = EMT::Ph3::PiLine::make("PiLine89_2", Logger::Level::debug);
	line89_2EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance89),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance89),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance89),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance89));
	//Line910
	auto line910EMT = EMT::Ph3::PiLine::make("PiLine910", Logger::Level::debug);
	line910EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance910),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance910),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance910),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance910));
	//Line1011
	auto line1011EMT = EMT::Ph3::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011EMT->setParameters(Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineResistance1011),
	                      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineInductance1011),
					      Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineCapacitance1011),
						  Math::singlePhaseParameterToThreePhase(KRK_TwoArea.lineConductance1011));

	// Topology
	gen1EMT->connect({ n5EMT });
	gen2EMT->connect({ n6EMT });
	gen3EMT->connect({ n11EMT });
	gen4EMT->connect({ n10EMT });

	// load7_1EMT->connect({ n7EMT });
	// load9_1EMT->connect({ n9EMT });
	// load7_2EMT->connect({ n7EMT });
	// load9_2EMT->connect({ n9EMT });

	load7EMT->connect({ n7EMT });
	load9EMT->connect({ n9EMT });

	line56EMT->connect({ n5EMT, n6EMT });
	line67EMT->connect({ n6EMT, n7EMT });
	line78_1EMT->connect({ n7EMT, n8EMT });
	line78_2EMT->connect({ n7EMT, n8EMT });
	line89_1EMT->connect({ n8EMT, n9EMT });
	line89_2EMT->connect({ n8EMT, n9EMT });
	line910EMT->connect({ n9EMT, n10EMT });
	line1011EMT->connect({ n10EMT, n11EMT });

	auto systemEMT = SystemTopology(60,
			SystemNodeList{n5EMT, n6EMT, n7EMT, n8EMT, n9EMT, n10EMT, n11EMT},
			SystemComponentList{gen1EMT, gen2EMT, gen3EMT, gen4EMT, load7EMT, load9EMT, line56EMT, line67EMT, line78_1EMT, line78_2EMT, line89_1EMT, line89_2EMT, line910EMT, line1011EMT});

	// Initialization of dynamic topology
	systemEMT.initWithPowerflow(systemPF);

	// Logging
	auto loggerEMT = DataLogger::make(simNameEMT);
	loggerEMT->addAttribute("v5", n5EMT->attribute("v"));
	loggerEMT->addAttribute("v6", n6EMT->attribute("v"));
	loggerEMT->addAttribute("v7", n7EMT->attribute("v"));
	loggerEMT->addAttribute("v8", n8EMT->attribute("v"));
	loggerEMT->addAttribute("v9", n9EMT->attribute("v"));
	loggerEMT->addAttribute("v10", n10EMT->attribute("v"));
	loggerEMT->addAttribute("v11", n11EMT->attribute("v"));
	loggerEMT->addAttribute("v_line56", line56EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line56", line56EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line67", line67EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line67", line67EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line78_1", line78_1EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line78_1", line78_1EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line78_2", line78_2EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line78_2", line78_2EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line89_1", line89_1EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line89_1", line89_1EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line89_2", line89_2EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line89_2", line89_2EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line910", line910EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line910", line910EMT->attribute("i_intf"));
	loggerEMT->addAttribute("v_line1011", line1011EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_line1011", line1011EMT->attribute("i_intf"));
	loggerEMT->addAttribute("Te_gen1", gen1EMT->attribute("T_e"));
	loggerEMT->addAttribute("v_gen1", gen1EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen1", gen1EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen1", gen1EMT->attribute("w_r"));
	// loggerEMT->addAttribute("wref_gen1", gen1EMT->attribute("w_ref"));
	loggerEMT->addAttribute("delta_gen1", gen1EMT->attribute("delta_r"));
	// loggerEMT->addAttribute("deltaref_gen1", gen1EMT->attribute("delta_ref"));
	loggerEMT->addAttribute("Te_gen2", gen2EMT->attribute("T_e"));
	loggerEMT->addAttribute("v_gen2", gen2EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen2", gen2EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen2", gen2EMT->attribute("w_r"));
	// loggerEMT->addAttribute("wref_gen2", gen2EMT->attribute("w_ref"));
	loggerEMT->addAttribute("delta_gen2", gen2EMT->attribute("delta_r"));
	// loggerEMT->addAttribute("deltaref_gen2", gen2EMT->attribute("delta_ref"));
	loggerEMT->addAttribute("Te_gen3", gen3EMT->attribute("T_e"));
	loggerEMT->addAttribute("v_gen3", gen3EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen3", gen3EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen3", gen3EMT->attribute("w_r"));
	loggerEMT->addAttribute("delta_gen3", gen3EMT->attribute("delta_r"));
	loggerEMT->addAttribute("Te_gen4", gen4EMT->attribute("T_e"));
	loggerEMT->addAttribute("v_gen4", gen4EMT->attribute("v_intf"));
	loggerEMT->addAttribute("i_gen4", gen4EMT->attribute("i_intf"));
	loggerEMT->addAttribute("wr_gen4", gen4EMT->attribute("w_r"));
	// loggerEMT->addAttribute("wref_gen2", gen2EMT->attribute("w_ref"));
	loggerEMT->addAttribute("delta_gen4", gen4EMT->attribute("delta_r"));
	// loggerEMT->addAttribute("deltaref_gen4", gen4EMT->attribute("delta_ref"));
	// loggerEMT->addAttribute("v_load7_1", load7_1EMT->attribute("v_intf"));
	// loggerEMT->addAttribute("i_load7_1", load7_1EMT->attribute("i_intf"));
	// loggerEMT->addAttribute("v_load9_1", load9_1EMT->attribute("v_intf"));
	// loggerEMT->addAttribute("i_load9_1", load9_1EMT->attribute("i_intf"));
	// loggerEMT->addAttribute("v_load7_2", load7_2EMT->attribute("v_intf"));
	// loggerEMT->addAttribute("i_load7_2", load7_2EMT->attribute("i_intf"));
	// loggerEMT->addAttribute("v_load9_2", load9_2EMT->attribute("v_intf"));
	// loggerEMT->addAttribute("i_load9_2", load9_2EMT->attribute("i_intf"));
	loggerEMT->addAttribute("Tm_gen1", gen1EMT->attribute("T_m"));
	loggerEMT->addAttribute("Tm_gen2", gen2EMT->attribute("T_m"));
	// loggerEMT->addAttribute("P_elec1", gen1EMT->attribute("P_elec"));
	// loggerEMT->addAttribute("P_elec2", gen2EMT->attribute("P_elec"));
	loggerEMT->addAttribute("Tm_gen3", gen3EMT->attribute("T_m"));
	loggerEMT->addAttribute("Tm_gen4", gen4EMT->attribute("T_m"));
	// loggerEMT->addAttribute("P_elec3", gen3EMT->attribute("P_elec"));
	// loggerEMT->addAttribute("P_elec4", gen4EMT->attribute("P_elec"));

	Simulation simEMT(simNameEMT, Logger::Level::debug);
	simEMT.setSystem(systemEMT);
	simEMT.setTimeStep(timeStep);
	simEMT.setFinalTime(finalTime);
	simEMT.setDomain(Domain::EMT);
	simEMT.addLogger(loggerEMT);
	simEMT.doSystemMatrixRecomputation(true);
	simEMT.setMnaSolverImplementation(MnaSolverFactory::MnaSolverImpl::EigenSparse);
	simEMT.run();
}

int main(int argc, char* argv[]) {


	//Simulation parameters
	String simName="VBR_SynGenDQ7odTrapez_KRK_TwoArea_SteadyState";
	Real finalTime = 2;
	Real timeStep = 50e-6;
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

	VBR_SynGenDQ7odTrapez_KRK_TwoArea_SteadyState(simName, timeStep, finalTime, startFaultEvent, endFaultEvent, startTimeFault, endTimeFault, cmdInertia_G1, cmdInertia_G2, cmdInertia_G3, cmdInertia_G4, cmdDamping_G1, cmdDamping_G2, cmdDamping_G3, cmdDamping_G4);
}
