/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include <DPsim.h>
#include "../Examples.h"

using namespace DPsim;
using namespace CPS;
using namespace CPS::CIM;

int main(int argc, char* argv[]) {

	Examples::SGIB::ScenarioConfig scenario;
	
	// Real finalTime = 0.003;
	Real finalTime = 10;
	Real timeStep = 0.001;
	CommandLineArgs args(argc, argv);
	if (argc > 1) {
		timeStep = args.timeStep;
		finalTime = args.duration;
	}

	// ----- POWERFLOW FOR INITIALIZATION -----
	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = "DP_Slack_PiLine_VSI_with_PF_Init_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetPF = SP::Ph1::externalGridInjection::make("Slack", Logger::Level::debug);
	extnetPF->setParameters(scenario.systemNominalVoltage);
	extnetPF->setBaseVoltage(scenario.systemNominalVoltage);
	extnetPF->modifyPowerFlowBusType(PowerflowBusType::VD);

	auto linePF = SP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	linePF->setParameters(scenario.lineResistance, scenario.lineInductance, 0);
	linePF->setBaseVoltage(scenario.systemNominalVoltage);

	auto loadPF = SP::Ph1::Load::make("Load", Logger::Level::debug);
	loadPF->setParameters(-scenario.pvNominalActivePower, -scenario.pvNominalReactivePower, scenario.systemNominalVoltage);
	loadPF->modifyPowerFlowBusType(PowerflowBusType::PQ); 

	// Topology
	extnetPF->connect({ n1PF });
	linePF->connect({ n1PF, n2PF });
	loadPF->connect({ n2PF });	
	auto systemPF = SystemTopology(50,
			SystemNodeList{n1PF, n2PF},
			SystemComponentList{extnetPF, linePF, loadPF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v1", n1PF->attribute("v"));
	loggerPF->addAttribute("v2", n2PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::debug);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doPowerFlowInit(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- DYNAMIC SIMULATION -----
	Real timeStepDP = timeStep;
	Real finalTimeDP = finalTime+timeStepDP;
	String simNameDP = "DP_Slack_PiLine_VSI_with_PF_Init_DP";
	Logger::setLogDir("logs/" + simNameDP);

	// Components
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);

	auto extnetDP = DP::Ph1::NetworkInjection::make("Slack", Logger::Level::debug);
	extnetDP->setParameters(Complex(scenario.systemNominalVoltage,0));

	auto lineDP = DP::Ph1::PiLine::make("PiLine", Logger::Level::debug);
	lineDP->setParameters(scenario.lineResistance, scenario.lineInductance);

	
	auto pv = DP::Ph1::AvVoltageSourceInverterDQ::make("pv", "pv", Logger::Level::debug, true);
	pv->setParameters(scenario.systemOmega, scenario.pvNominalVoltage, scenario.pvNominalActivePower, scenario.pvNominalReactivePower);
	pv->setControllerParameters(scenario.KpPLL, scenario.KiPLL, scenario.KpPowerCtrl, scenario.KiPowerCtrl, scenario.KpCurrCtrl, scenario.KiCurrCtrl, scenario.OmegaCutoff);
	pv->setFilterParameters(scenario.Lf, scenario.Cf, scenario.Rf, scenario.Rc);
	pv->setTransformerParameters(scenario.systemNominalVoltage, scenario.pvNominalVoltage, scenario.transformerNominalPower, scenario.systemNominalVoltage/scenario.pvNominalVoltage, 0, 0, scenario.transformerInductance, scenario.systemOmega);
	pv->setInitialStateValues(scenario.pvNominalActivePower, scenario.pvNominalReactivePower, scenario.phi_dInit, scenario.phi_qInit, scenario.gamma_dInit, scenario.gamma_qInit);

	// Topology
	extnetDP->connect({ n1DP });
	lineDP->connect({ n1DP, n2DP });
	pv->connect({ n2DP });	
	auto systemDP = SystemTopology(50,
			SystemNodeList{n1DP, n2DP},
			SystemComponentList{extnetDP, lineDP, pv});

	// Initialization of dynamic topology with values from powerflow
	CIM::Reader reader(simNameDP, Logger::Level::debug);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);			

	// Logging
	auto loggerDP = DataLogger::make(simNameDP);
	loggerDP->addAttribute("v1", n1DP->attribute("v"));
	loggerDP->addAttribute("v2", n2DP->attribute("v"));

	Examples::CIGREMV::logPVDecomposedAttributes(loggerDP, pv);

	// load step sized in absolute terms
	std::shared_ptr<SwitchEvent> loadStepEvent = Examples::createEventAddPowerConsumption("n2", 1-timeStepDP, 100.0e3, systemDP, Domain::DP, loggerDP);

	// Simulation
	Simulation sim(simNameDP, Logger::Level::debug);
	sim.setSystem(systemDP);
	sim.setTimeStep(timeStepDP);
	sim.setFinalTime(finalTimeDP);
	sim.setDomain(Domain::DP);
	sim.addEvent(loadStepEvent);
	sim.doPowerFlowInit(false);
	sim.addLogger(loggerDP);
	sim.run();
	
}
