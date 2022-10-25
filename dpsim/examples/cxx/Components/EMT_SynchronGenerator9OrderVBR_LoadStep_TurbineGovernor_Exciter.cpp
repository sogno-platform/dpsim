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

// Parameters synchronous generator
const Examples::Components::SynchronousGeneratorKundur::MachineParameters syngenKundur;
const Examples::Components::TurbineGovernor::GovernorKundur govKundur;

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
	String simName="EMT_Ph3_SynchronGenerator9OrderVBR_LoadStep_TurbineGovernor_Exciter";
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
			cmdInertiaFactor = args.getOptionReal("SCALEINERTIA");
		if (args.options.find("WITHGOVERNOR") != args.options.end())
			withGovernor = args.getOptionBool("WITHGOVERNOR");
		if (args.options.find("WITHEXCITER") != args.options.end())
			withExciter = args.getOptionBool("WITHEXCITER");
		if (args.options.find("TIMESTEPEVENT") != args.options.end())
			timeStepEvent = args.getOptionReal("TIMESTEPEVENT");
		if (args.options.find("LOADFACTOR") != args.options.end())
			cmdLoadFactor = args.getOptionReal("LOADFACTOR");
	}

	// Calculate grid parameters
	Real RloadOriginal = std::pow(nominalVoltage,2)/PloadOriginal;
	Real RloadNew = std::pow(nominalVoltage,2)/(cmdLoadFactor*PloadOriginal);

	// Set logger directory
	Logger::setLogDir("logs/"+simName);

	// Nodes
	auto n1 = CPS::EMT::SimNode::make("n1", PhaseType::ABC);

	// Components
	auto gen = CPS::EMT::Ph3::SynchronGeneratorVBR::make("SynGen", CPS::Logger::Level::info);
	gen->setBaseAndFundamentalPerUnitParameters(syngenKundur.nomPower, syngenKundur.nomVoltage, syngenKundur.nomFreq, syngenKundur.poleNum, syngenKundur.nomFieldCurr,
												syngenKundur.Rs, syngenKundur.Ll, syngenKundur.Ld, syngenKundur.Lq, syngenKundur.Rfd, syngenKundur.Llfd,
												syngenKundur.Rkd, syngenKundur.Llkd, syngenKundur.Rkq1, syngenKundur.Llkq1, syngenKundur.Rkq2, syngenKundur.Llkq2, cmdInertiaFactor*syngenKundur.H);
	gen->setInitialValues(initActivePower, initReactivePower, initTerminalVoltageMagnitude,
		initTerminalVoltageAngle, initMechPower);

	if (withGovernor)
		gen->addGovernor(govKundur.Ta_t, govKundur.Tb, govKundur.Tc, govKundur.Fa, govKundur.Fb, govKundur.Fc, govKundur.Kg, govKundur.Tsr, govKundur.Tsm, initActivePower / syngenKundur.nomPower, initMechPower / syngenKundur.nomPower);

	std::shared_ptr<Base::Exciter> exciter = nullptr;
	if (withExciter) {
		exciter = CPS::Signal::ExciterDC1Simp::make("Exciter", CPS::Logger::Level::info);
		exciter->setParameters(Examples::Components::Exciter::getExciterEremia());
		gen->addExciter(exciter);
	}

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
	logger->logAttribute("v1", n1->attribute("v"));
	logger->logAttribute("i_gen", gen->attribute("i_intf"));
	logger->logAttribute("wr_gen", gen->attribute("w_r"));
	logger->logAttribute("vf_exc_gen", gen->attribute("Vfd"));

	Simulation sim(simName, Logger::Level::info);
	sim.setSystem(sys);
	sim.setTimeStep(timeStep);
	sim.setFinalTime(finalTime);
	sim.doSystemMatrixRecomputation(true);
	sim.setDomain(Domain::EMT);
	sim.addLogger(logger);

	// Events
	auto sw1 = SwitchEvent3Ph::make(timeStepEvent, fault, true);
	sim.addEvent(sw1);

	sim.run();
}
