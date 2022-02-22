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
using namespace CPS::CIM;

// Machine parameters synchronous generator
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;

// Initialization parameters
Real nominalVoltage = 24e3;
Real initActivePower = 300e6;
Real initReactivePower = 0;
Real initMechPower = 300e6;
Real initTerminalVoltageMagnitude = nominalVoltage*RMS3PH_TO_PEAK1PH;
Real initTerminalVoltageAngle = -PI / 2;
Complex initTerminalVoltage = CPS::Math::polar(initTerminalVoltageMagnitude, initTerminalVoltageAngle);

// Define load parameters
Real PloadOriginal = 300e6;

int main(int argc, char* argv[]) {

	//Simulation parameters
	String simName="EMT_Ph3_SynchronGenerator9OrderDCIM_LoadStep_TurbineGovernor_Exciter";
	Real timeStep = 50e-6;
	Real finalTime = 1.0;
	Real cmdInertiaFactor= 1.0;
	Bool withGovernor = false;
	Bool withExciter = false;
	Real timeStepEvent = 30.0;
	Real cmdLoadFactor = 1.5;
	

	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
		if (args.name != "dpsim")
			simName = args.name;
		if (args.options.find("SCALEINERTIA") != args.options.end())
			cmdInertiaFactor = args.options["SCALEINERTIA"];
		if (args.options_bool.find("WITHGOVERNOR") != args.options_bool.end())
			withGovernor = args.options_bool["WITHGOVERNOR"];
		if (args.options_bool.find("WITHEXCITER") != args.options_bool.end())
			withExciter = args.options_bool["WITHEXCITER"];
		if (args.options.find("TIMESTEPEVENT") != args.options.end())
			timeStepEvent = args.options["TIMESTEPEVENT"];
		if (args.options.find("LOADFACTOR") != args.options.end())
			cmdLoadFactor = args.options["LOADFACTOR"];
	}

	// Calculate grid parameters
	Real RloadOriginal = std::pow(nominalVoltage,2)/PloadOriginal;
	Real RloadNew = std::pow(nominalVoltage,2)/(cmdLoadFactor*PloadOriginal);

	// Set logger directory
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::SimNode::make("n1", PhaseType::ABC);

	// Components
	auto gen = CPS::EMT::Ph3::SynchronGeneratorDQTrapez::make("SynGen", CPS::Logger::Level::info);
	gen->setParametersFundamentalPerUnit(syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, syngenKundur.poleNum, syngenKundur.nomFieldCurr,
		syngenKundur.Rs, syngenKundur.Ll, syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Rfd, syngenKundur.Llfd, syngenKundur.Rkd, syngenKundur.Llkd, syngenKundur.Rkq1, syngenKundur.Llkq1, syngenKundur.Rkq2, syngenKundur.Llkq2, cmdInertiaFactor*syngenKundur.H,
		initActivePower, initReactivePower, initTerminalVoltageMagnitude,
		initTerminalVoltageAngle, initMechPower);

	if (withGovernor) 
		gen->addGovernor(syngenKundur.Ta_t, syngenKundur.Tb, syngenKundur.Tc, syngenKundur.Fa, syngenKundur.Fb, syngenKundur.Fc, syngenKundur.Kg, syngenKundur.Tsr, syngenKundur.Tsm, initActivePower / syngenKundur.nomPower, initMechPower / syngenKundur.nomPower);
	
	if (withExciter)
		gen->addExciter(syngenKundur.Ta, syngenKundur.Ka, syngenKundur.Te, syngenKundur.Ke, syngenKundur.Tf, syngenKundur.Kf, syngenKundur.Tr);

	auto fault = CPS::EMT::Ph3::Switch::make("Br_fault", CPS::Logger::Level::info);
	fault->setParameters(Math::singlePhaseParameterToThreePhase(RloadOriginal), 
						 Math::singlePhaseParameterToThreePhase(RloadNew));
	fault->openSwitch();


	// Connections
	gen->connect({n1});
	fault->connect({CPS::EMT::SimNode::GND, n1});

	auto sys = SystemTopology(60, SystemNodeList{n1}, SystemComponentList{gen, fault});

	// Logging
	auto logger = DataLogger::make(simName);
	logger->addAttribute("v1", n1->attribute("v"));
	logger->addAttribute("i_gen", gen->attribute("i_intf"));
	logger->addAttribute("wr_gen", gen->attribute("w_r"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	// Events
	auto sw1 = SwitchEvent3Ph::make(timeStepEvent, fault, true);
	sim.addEvent(sw1);

	sim.run();
}
