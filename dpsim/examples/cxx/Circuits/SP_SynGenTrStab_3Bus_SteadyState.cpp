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
using namespace CIM::Examples::Grids::ThreeBus;

ScenarioConfig ThreeBus;

void SP_SynGenTrStab_3Bus_SteadyState(String simName, Real timeStep, Real finalTime, Real cmdInertia_G1, Real cmdInertia_G2, Real cmdDamping_G1, Real cmdDamping_G2) {
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
	auto gen1PF = SP::Ph1::SynchronGenerator::make("SynGen1", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.initActivePower_G1, ThreeBus.setPointVoltage_G1*ThreeBus.t1_ratio, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(ThreeBus.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("SynGen2", Logger::Level::debug);
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
	loggerPF->logAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->logAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->logAttribute("v_bus3", n3PF->attribute("v"));

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

	// ----- Dynamic simulation ------
	String simNameSP = simName + "_SP";
	Logger::setLogDir("logs/"+simNameSP);

	// Nodes
	auto n1SP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2SP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3SP = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	//Synchronous generator 1
	auto gen1SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen1", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1SP->setStandardParametersPU(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.nomFreq_G1, ThreeBus.Xpd_G1*std::pow(ThreeBus.t1_ratio,2), cmdInertia_G1*ThreeBus.H_G1, ThreeBus.Rs_G1, cmdDamping_G1*ThreeBus.D_G1);

	//Synchronous generator 2
	auto gen2SP = SP::Ph1::SynchronGeneratorTrStab::make("SynGen2", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2SP->setStandardParametersPU(ThreeBus.nomPower_G2, ThreeBus.nomPhPhVoltRMS_G2, ThreeBus.nomFreq_G2, ThreeBus.Xpd_G2*std::pow(ThreeBus.t2_ratio,2), cmdInertia_G2*ThreeBus.H_G2, ThreeBus.Rs_G2, cmdDamping_G2*ThreeBus.D_G2);

	//Load
	auto loadSP = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadSP->setParameters(ThreeBus.activePower_L, ThreeBus.reactivePower_L, ThreeBus.Vnom);

	//Line12
	auto line12SP = SP::Ph1::PiLine::make("PiLine12", Logger::Level::debug);
	line12SP->setParameters(ThreeBus.lineResistance12, ThreeBus.lineInductance12, ThreeBus.lineCapacitance12, ThreeBus.lineConductance12);
	//Line13
	auto line13SP = SP::Ph1::PiLine::make("PiLine13", Logger::Level::debug);
	line13SP->setParameters(ThreeBus.lineResistance13, ThreeBus.lineInductance13, ThreeBus.lineCapacitance13, ThreeBus.lineConductance13);
	//Line23
	auto line23SP = SP::Ph1::PiLine::make("PiLine23", Logger::Level::debug);
	line23SP->setParameters(ThreeBus.lineResistance23, ThreeBus.lineInductance23, ThreeBus.lineCapacitance23, ThreeBus.lineConductance23);

	// Topology
	gen1SP->connect({ n1SP });
	gen2SP->connect({ n2SP });
	loadSP->connect({ n3SP });
	line12SP->connect({ n1SP, n2SP });
	line13SP->connect({ n1SP, n3SP });
	line23SP->connect({ n2SP, n3SP });
	auto systemSP = SystemTopology(60,
			SystemNodeList{n1SP, n2SP, n3SP},
			SystemComponentList{gen1SP, gen2SP, loadSP, line12SP, line13SP, line23SP});

	// Initialization of dynamic topology
	systemSP.initWithPowerflow(systemPF);

	// Logging
	auto loggerSP = DataLogger::make(simNameSP);
	loggerSP->logAttribute("v1", n1SP->attribute("v"));
	loggerSP->logAttribute("v2", n2SP->attribute("v"));
	loggerSP->logAttribute("v3", n3SP->attribute("v"));
	loggerSP->logAttribute("v_line12", line12SP->attribute("v_intf"));
	loggerSP->logAttribute("i_line12", line12SP->attribute("i_intf"));
	loggerSP->logAttribute("v_line13", line13SP->attribute("v_intf"));
	loggerSP->logAttribute("i_line13", line13SP->attribute("i_intf"));
	loggerSP->logAttribute("v_line23", line23SP->attribute("v_intf"));
	loggerSP->logAttribute("i_line23", line23SP->attribute("i_intf"));
	loggerSP->logAttribute("Ep_gen1", gen1SP->attribute("Ep_mag"));
	loggerSP->logAttribute("v_gen1", gen1SP->attribute("v_intf"));
	loggerSP->logAttribute("i_gen1", gen1SP->attribute("i_intf"));
	loggerSP->logAttribute("wr_gen1", gen1SP->attribute("w_r"));
	loggerSP->logAttribute("delta_gen1", gen1SP->attribute("delta_r"));
	loggerSP->logAttribute("Ep_gen2", gen2SP->attribute("Ep_mag"));
	loggerSP->logAttribute("v_gen2", gen2SP->attribute("v_intf"));
	loggerSP->logAttribute("i_gen2", gen2SP->attribute("i_intf"));
	loggerSP->logAttribute("wr_gen2", gen2SP->attribute("w_r"));
	loggerSP->logAttribute("wref_gen2", gen2SP->attribute("w_ref"));
	loggerSP->logAttribute("delta_gen2", gen2SP->attribute("delta_r"));
	loggerSP->logAttribute("v_load", loadSP->attribute("v_intf"));
	loggerSP->logAttribute("i_load", loadSP->attribute("i_intf"));
	loggerSP->logAttribute("P_mech1", gen1SP->attribute("P_mech"));
	loggerSP->logAttribute("P_mech2", gen2SP->attribute("P_mech"));
	loggerSP->logAttribute("P_elec1", gen1SP->attribute("P_elec"));
	loggerSP->logAttribute("P_elec2", gen2SP->attribute("P_elec"));

	Simulation sim(simNameSP, Logger::Level::debug);
	sim.setSystem(systemSP);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::SP);
	sim.addLogger(loggerSP);

	sim.run();
}

int main(int argc, char* argv[]) {


	//Simultion parameters
	String simName="SP_SynGenTrStab_3Bus_SteadyState";
	Real finalTime = 30;
	Real timeStep = 0.001;
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
			cmdInertia_G1 = args.getOptionReal("SCALEINERTIA_G1");
		if (args.options.find("SCALEINERTIA_G2") != args.options.end())
			cmdInertia_G2 = args.getOptionReal("SCALEINERTIA_G2");
		if (args.options.find("SCALEDAMPING_G1") != args.options.end())
			cmdDamping_G1 = args.getOptionReal("SCALEDAMPING_G1");
		if (args.options.find("SCALEDAMPING_G2") != args.options.end())
			cmdDamping_G2 = args.getOptionReal("SCALEDAMPING_G2");
	}

	SP_SynGenTrStab_3Bus_SteadyState(simName, timeStep, finalTime, cmdInertia_G1, cmdInertia_G2, cmdDamping_G1, cmdDamping_G2);
}
