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
using namespace CIM::Examples::Grids::SMIB;

ScenarioConfig smib;

void DP_1ph_SynGenTrStab_SteadyState(String simName, Real timeStep, Real finalTime, Real cmdInertia, Real cmdDamping) {
	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	//Synchronous generator ideal model
	auto genPF = SP::Ph1::SynchronGenerator::make("SynGen", Logger::Level::debug);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	genPF->setParameters(smib.nomPower, smib.nomPhPhVoltRMS, smib.initActivePower, smib.setPointVoltage*smib.t_ratio, PowerflowBusType::PV);
	genPF->setBaseVoltage(smib.Vnom);
	genPF->modifyPowerFlowBusType(PowerflowBusType::PV);

	//Grid bus as Slack
	auto extnetPF = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(smib.Vnom);
	extnetPF->setBaseVoltage(smib.Vnom);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	//Line
	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(smib.lineResistance, smib.lineInductance, smib.lineCapacitance, smib.lineConductance);
	linePF->setBaseVoltage(smib.Vnom);

	// Topology
	genPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	extnetPF->connect({ n2PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{genPF, linePF, extnetPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->logAttribute("v1", n1PF->attribute("v"));
	loggerPF->logAttribute("v2", n2PF->attribute("v"));

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
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/"+simNameDP);

	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	// Components
	auto genDP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen", Logger::Level::debug);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	genDP->setStandardParametersPU(smib.nomPower, smib.nomPhPhVoltRMS, smib.nomFreq, smib.Xpd*std::pow(smib.t_ratio,2), cmdInertia*smib.H, smib.Rs, cmdDamping*smib.D );

	//Grid bus as Slack
	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetDP->setParameters(smib.Vnom);
	// Line
	auto lineDP = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineDP->setParameters(smib.lineResistance, smib.lineInductance, smib.lineCapacitance, smib.lineConductance);

	// Topology
	genDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	extnetDP->connect({ n2DP });
	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{genDP, lineDP, extnetDP});

	// Initialization of dynamic topology
	systemDP.initWithPowerflow(systemPF);


	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->logAttribute("v1", n1DP->attribute("v"));
	loggerDP->logAttribute("v2", n2DP->attribute("v"));
	//gen
	loggerDP->logAttribute("Ep", genDP->attribute("Ep"));
	loggerDP->logAttribute("v_gen", genDP->attribute("v_intf"));
	loggerDP->logAttribute("i_gen", genDP->attribute("i_intf"));
	loggerDP->logAttribute("wr_gen", genDP->attribute("w_r"));
	loggerDP->logAttribute("delta_r_gen", genDP->attribute("delta_r"));
	loggerDP->logAttribute("P_elec", genDP->attribute("P_elec"));
	loggerDP->logAttribute("P_mech", genDP->attribute("P_mech"));
	//line
	loggerDP->logAttribute("v_line", lineDP->attribute("v_intf"));
	loggerDP->logAttribute("i_line", lineDP->attribute("i_intf"));
	//slack
	loggerDP->logAttribute("v_slack", extnetDP->attribute("v_intf"));
	loggerDP->logAttribute("i_slack", extnetDP->attribute("i_intf"));



	Simulation simDP(simNameDP, Logger::Level::debug);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.addLogger(loggerDP);

	simDP.run();
}

int main(int argc, char* argv[]) {


	//Simulation parameters
	String simName="DP_SynGenTrStab_SMIB_SteadyState";
	Real finalTime = 20;
	Real timeStep = 0.001;
	Real cmdInertia= 1.0;
	Real cmdDamping=1.0;

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA") != args.options.end())
			cmdInertia = args.getOptionReal("SCALEINERTIA");
		if (args.options.find("SCALEDAMPING") != args.options.end())
			cmdDamping = args.getOptionReal("SCALEDAMPING");
	}

	DP_1ph_SynGenTrStab_SteadyState(simName, timeStep, finalTime, cmdInertia, cmdDamping);
}
