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

ScenarioConfig KRK_TwoArea;

//Switch to trigger fault at generator terminal
Real SwitchOpen = 1e12;
Real SwitchClosed = 0.1;

void DP_SynGenTrStab_KRK_TwoArea_Fault(String simName, Real timeStep, Real finalTime, bool startFaultEvent, bool endFaultEvent, Real startTimeFault, Real endTimeFault, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdInertia_G3, Real cmdInertia_G4, Real cmdDamping_G1, Real cmdDamping_G2, Real cmdDamping_G3, Real cmdDamping_G4) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);
    auto n4PF = SimNode<Complex>::make("n4", PhaseType::Single);
	auto n5PF = SimNode<Complex>::make("n5", PhaseType::Single);
	auto n6PF = SimNode<Complex>::make("n6", PhaseType::Single);
    auto n7PF = SimNode<Complex>::make("n7", PhaseType::Single);
	auto n8PF = SimNode<Complex>::make("n8", PhaseType::Single);
	auto n9PF = SimNode<Complex>::make("n9", PhaseType::Single);
    auto n10PF = SimNode<Complex>::make("n10", PhaseType::Single);
	auto n11PF = SimNode<Complex>::make("n11", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.initActivePower_G1, KRK_TwoArea.setPointVoltage_G1*KRK_TwoArea.t1_ratio, PowerflowBusType::PV);
	gen1PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.initActivePower_G2, KRK_TwoArea.setPointVoltage_G2*KRK_TwoArea.t2_ratio, PowerflowBusType::VD);
	gen2PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 3
	auto gen3PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen3PF->setParameters(KRK_TwoArea.nomPower_G3, KRK_TwoArea.nomPhPhVoltRMS_G3, KRK_TwoArea.initActivePower_G3, KRK_TwoArea.setPointVoltage_G3*KRK_TwoArea.t3_ratio, PowerflowBusType::PV);
	gen3PF->setBaseVoltage(KRK_TwoArea.Vnom);

    //Synchronous generator 4
	auto gen4PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen4PF->setParameters(KRK_TwoArea.nomPower_G4, KRK_TwoArea.nomPhPhVoltRMS_G4, KRK_TwoArea.initActivePower_G4, KRK_TwoArea.setPointVoltage_G4*KRK_TwoArea.t4_ratio, PowerflowBusType::PV);
	gen4PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//use Shunt as Load for powerflow
	auto load7PF = SP::Ph1::Shunt::make("Load7", Logger::Level::debug);
	load7PF->setParameters(KRK_TwoArea.activePower_L7 / std::pow(KRK_TwoArea.Vnom, 2), - KRK_TwoArea.reactivePower_L7_inductive / std::pow(KRK_TwoArea.Vnom, 2));
	load7PF->setBaseVoltage(KRK_TwoArea.Vnom);

    auto load9PF = SP::Ph1::Shunt::make("Load9", Logger::Level::debug);
	load9PF->setParameters(KRK_TwoArea.activePower_L9 / std::pow(KRK_TwoArea.Vnom, 2), - KRK_TwoArea.reactivePower_L9_inductive / std::pow(KRK_TwoArea.Vnom, 2));
	load9PF->setBaseVoltage(KRK_TwoArea.Vnom);

	//Line56
	auto line56PF = SP::Ph1::PiLine::make("PiLine56", Logger::Level::debug);
	line56PF->setParameters(KRK_TwoArea.lineResistance56, KRK_TwoArea.lineInductance56, KRK_TwoArea.lineCapacitance56, KRK_TwoArea.lineConductance56);
	line56PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line67
	auto line67PF = SP::Ph1::PiLine::make("PiLine67", Logger::Level::debug);
	line67PF->setParameters(KRK_TwoArea.lineResistance67, KRK_TwoArea.lineInductance67, KRK_TwoArea.lineCapacitance67, KRK_TwoArea.lineConductance67);
	line67PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Line78
	auto line78PF = SP::Ph1::PiLine::make("PiLine78", Logger::Level::debug);
	line78PF->setParameters(KRK_TwoArea.lineResistance78, KRK_TwoArea.lineInductance78, KRK_TwoArea.lineCapacitance78, KRK_TwoArea.lineConductance78);
	line78PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line89
	auto line89PF = SP::Ph1::PiLine::make("PiLine89", Logger::Level::debug);
	line89PF->setParameters(KRK_TwoArea.lineResistance89, KRK_TwoArea.lineInductance89, KRK_TwoArea.lineCapacitance89, KRK_TwoArea.lineConductance89);
	line89PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line910
	auto line910PF = SP::Ph1::PiLine::make("PiLine910", Logger::Level::debug);
	line910PF->setParameters(KRK_TwoArea.lineResistance910, KRK_TwoArea.lineInductance910, KRK_TwoArea.lineCapacitance910, KRK_TwoArea.lineConductance910);
	line910PF->setBaseVoltage(KRK_TwoArea.Vnom);
    //Line1011
	auto line1011PF = SP::Ph1::PiLine::make("PiLine1011", Logger::Level::debug);
	line1011PF->setParameters(KRK_TwoArea.lineResistance1011, KRK_TwoArea.lineInductance1011, KRK_TwoArea.lineCapacitance1011, KRK_TwoArea.lineConductance1011);
	line1011PF->setBaseVoltage(KRK_TwoArea.Vnom);
	//Switch
	auto faultPF = CPS::SP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	faultPF->setParameters(SwitchOpen, SwitchClosed);
	faultPF->open();

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
    gen3PF->connect({ n3PF });
    gen4PF->connect({ n4PF });
	load7PF->connect({ n7PF });
    load9PF->connect({ n9PF });
	line56PF->connect({ n5PF, n6PF });
	line67PF->connect({ n6PF, n7PF });
	line78PF->connect({ n7PF, n8PF });
    line89PF->connect({ n8PF, n9PF });
    line910PF->connect({ n9PF, n10PF });
    line1011PF->connect({ n10PF, n11PF });
	// faultPF->connect({SP::SimNode::GND , n1PF }); //terminal of generator 1
	faultPF->connect({SP::SimNode::GND , n2PF }); //terminal of generator 2
	// faultPF->connect({SP::SimNode::GND , n3PF }); //Load bus
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF, n4PF, n5PF, n6PF, n7PF, n8PF, n9PF, n10PF, n11PF},
			SystemComponentList{gen1PF, gen2PF, gen3PF, load7PF, load9PF, line56PF, line67PF, line78PF, line89PF, line910PF, line1011PF, faultPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));
    loggerPF->addAttribute("v_bus4", n4PF->attribute("v"));
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
	// String simNameDP = simName + "_DP";
	// Logger::setLogDir("logs/"+simNameDP);

	// // Nodes
	// auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	// auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	// auto n3DP = SimNode<Complex>::make("n3", PhaseType::Single);

	// // Components
	// //Synchronous generator 1
	// auto gen1DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen1", Logger::Level::debug);
	// // Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	// gen1DP->setStandardParametersPU(KRK_TwoArea.nomPower_G1, KRK_TwoArea.nomPhPhVoltRMS_G1, KRK_TwoArea.nomFreq_G1, KRK_TwoArea.Xpd_G1*std::pow(KRK_TwoArea.t1_ratio,2), cmdInertia_G1*KRK_TwoArea.H_G1, KRK_TwoArea.Rs_G1, cmdDamping_G1*KRK_TwoArea.D_G1);
	// // Get actual active and reactive power of generator's Terminal from Powerflow solution
	// Complex initApparentPower_G1= gen1PF->getApparentPower();
	// gen1DP->setInitialValues(initApparentPower_G1, KRK_TwoArea.initMechPower_G1);

	// //Synchronous generator 2
	// auto gen2DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen2", Logger::Level::debug);
	// // Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	// gen2DP->setStandardParametersPU(KRK_TwoArea.nomPower_G2, KRK_TwoArea.nomPhPhVoltRMS_G2, KRK_TwoArea.nomFreq_G2, KRK_TwoArea.Xpd_G2*std::pow(KRK_TwoArea.t2_ratio,2), cmdInertia_G2*KRK_TwoArea.H_G2, KRK_TwoArea.Rs_G2, cmdDamping_G2*KRK_TwoArea.D_G2);
	// // Get actual active and reactive power of generator's Terminal from Powerflow solution
	// Complex initApparentPower_G2= gen2PF->getApparentPower();
	// gen2DP->setInitialValues(initApparentPower_G2, KRK_TwoArea.initMechPower_G2);

	// gen2DP->setModelFlags(true, true);
	// gen2DP->setReferenceOmega(gen1DP->attribute<Real>("w_r"), gen1DP->attribute<Real>("delta_r"));

	// ///Load
	// auto loadDP=DP::Ph1::RXLoad::make("Load", Logger::Level::debug);
	// loadDP->setParameters(KRK_TwoArea.activePower_L, KRK_TwoArea.reactivePower_L, KRK_TwoArea.Vnom);

	// //Line12
	// auto line12DP = DP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	// line12DP->setParameters(KRK_TwoArea.lineResistance12, KRK_TwoArea.lineInductance12, KRK_TwoArea.lineCapacitance12, KRK_TwoArea.lineConductance12);
	// //Line13
	// auto line13DP = DP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	// line13DP->setParameters(KRK_TwoArea.lineResistance13, KRK_TwoArea.lineInductance13, KRK_TwoArea.lineCapacitance13, KRK_TwoArea.lineConductance13);
	// //Line23
	// auto line23DP = DP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	// line23DP->setParameters(KRK_TwoArea.lineResistance23, KRK_TwoArea.lineInductance23, KRK_TwoArea.lineCapacitance23, KRK_TwoArea.lineConductance23);

	// // //Switch
	// // auto faultDP = DP::Ph1::Switch::make("Br_fault", Logger::Level::debug);
	// // faultDP->setParameters(SwitchOpen, SwitchClosed);
	// // faultDP->open();

	// //Variable resistance Switch
	// auto faultDP = DP::Ph1::varResSwitch::make("Br_fault", Logger::Level::debug);
	// faultDP->setParameters(SwitchOpen, SwitchClosed);
	// faultDP->setInitParameters(timeStep);
	// faultDP->open();

	// // Topology
	// gen1DP->connect({ n1DP });
	// gen2DP->connect({ n2DP });
	// loadDP->connect({ n3DP });
	// line12DP->connect({ n1DP, n2DP });
	// line13DP->connect({ n1DP, n3DP });
	// line23DP->connect({ n2DP, n3DP });
	// // faultDP->connect({DP::SimNode::GND , n1DP }); //terminal of generator 1
	// faultDP->connect({DP::SimNode::GND , n2DP }); //terminal of generator 2
	// // faultDP->connect({DP::SimNode::GND , n3DP }); //Load bus
	// auto systemDP = SystemTopology(60,
	// 		SystemNodeList{n1DP, n2DP, n3DP},
	// 		SystemComponentList{gen1DP, gen2DP, loadDP, line12DP, line13DP, line23DP, faultDP});

	// // Initialization of dynamic topology
	// CIM::Reader reader(simNameDP, Logger::Level::debug);
	// reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);

	// // Logging
	// auto loggerDP = DataLogger::make(simNameDP);
	// loggerDP->addAttribute("v1", n1DP->attribute("v"));
	// loggerDP->addAttribute("v2", n2DP->attribute("v"));
	// loggerDP->addAttribute("v3", n3DP->attribute("v"));
	// loggerDP->addAttribute("v_line12", line12DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line12", line12DP->attribute("i_intf"));
	// loggerDP->addAttribute("v_line13", line13DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line13", line13DP->attribute("i_intf"));
	// loggerDP->addAttribute("v_line23", line23DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_line23", line23DP->attribute("i_intf"));
	// loggerDP->addAttribute("Ep_gen1", gen1DP->attribute("Ep_mag"));
	// loggerDP->addAttribute("v_gen1", gen1DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_gen1", gen1DP->attribute("i_intf"));
	// loggerDP->addAttribute("wr_gen1", gen1DP->attribute("w_r"));
	// loggerDP->addAttribute("delta_gen1", gen1DP->attribute("delta_r"));
	// loggerDP->addAttribute("Ep_gen2", gen2DP->attribute("Ep_mag"));
	// loggerDP->addAttribute("v_gen2", gen2DP->attribute("v_intf"));
	// loggerDP->addAttribute("i_gen2", gen2DP->attribute("i_intf"));
	// loggerDP->addAttribute("wr_gen2", gen2DP->attribute("w_r"));
	// loggerDP->addAttribute("wref_gen2", gen2DP->attribute("w_ref"));
	// loggerDP->addAttribute("delta_gen2", gen2DP->attribute("delta_r"));
	// loggerDP->addAttribute("i_fault", faultDP->attribute("i_intf"));
	// loggerDP->addAttribute("v_load", loadDP->attribute("v_intf"));
	// loggerDP->addAttribute("i_load", loadDP->attribute("i_intf"));
	// loggerDP->addAttribute("P_mech1", gen1DP->attribute("P_mech"));
	// loggerDP->addAttribute("P_mech2", gen2DP->attribute("P_mech"));
	// loggerDP->addAttribute("P_elec1", gen1DP->attribute("P_elec"));
	// loggerDP->addAttribute("P_elec2", gen2DP->attribute("P_elec"));

	// Simulation simDP(simNameDP, Logger::Level::debug);
	// simDP.setSystem(systemDP);
	// simDP.setTimeStep(timeStep);
	// simDP.setFinalTime(finalTime);
	// simDP.setDomain(Domain::DP);
	// simDP.addLogger(loggerDP);
	// simDP.doSystemMatrixRecomputation(true);

	// // Events
	// if (startFaultEvent){
	// 	auto sw1 = SwitchEvent::make(startTimeFault, faultDP, true);

	// 	simDP.addEvent(sw1);
	// }

	// if(endFaultEvent){

	// 	auto sw2 = SwitchEvent::make(endTimeFault, faultDP, false);
	// 	simDP.addEvent(sw2);

	// }

	// simDP.run();
}

int main(int argc, char* argv[]) {


	//Simultion parameters
	String simName="DP_SynGenTrStab_KRK_TwoArea_Fault";
	Real finalTime = 30;
	Real timeStep = 0.001;
	Bool startFaultEvent=true;
	Bool endFaultEvent=true;
	Real startTimeFault=10;
	Real endTimeFault=10.2;
	Real cmdInertia_G1= 1.0;
	Real cmdInertia_G2= 1.0;
    Real cmdInertia_G3= 1.0;
    Real cmdInertia_G4= 1.0;
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
