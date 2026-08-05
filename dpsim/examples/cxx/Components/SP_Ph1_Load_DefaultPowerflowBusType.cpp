// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

const String simName = "SP_Ph1_Load_DefaultPowerflowBusType";

const Real nominalVoltage = 20e3;
const Real loadActivePower = 100e3;
const Real loadReactivePower = 50e3;

Complex solveLoadBusVoltage(const String &caseName, Bool setBusTypeExplicitly) {
  Logger::setLogDir("logs/" + caseName);

  auto slackNode = SimNode<Complex>::make("n1", PhaseType::Single);
  auto loadNode = SimNode<Complex>::make("n2", PhaseType::Single);

  auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  slack->setParameters(nominalVoltage);
  slack->setBaseVoltage(nominalVoltage);
  slack->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto line = SP::Ph1::PiLine::make("PiLine", Logger::Level::off);
  line->setParameters(0.05, 0.1, 0.1e-6);
  line->setBaseVoltage(nominalVoltage);

  auto load = SP::Ph1::Load::make("Load", Logger::Level::off);
  load->setParameters(loadActivePower, loadReactivePower, nominalVoltage);

  if (setBusTypeExplicitly)
    load->modifyPowerFlowBusType(PowerflowBusType::PQ);

  slack->connect({slackNode});
  line->connect({slackNode, loadNode});
  load->connect({loadNode});

  auto system = SystemTopology(50, SystemNodeList{slackNode, loadNode},
                               SystemComponentList{slack, line, load});

  Simulation simulation(caseName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setTimeStep(1.0);
  simulation.setFinalTime(2.0);
  simulation.setDomain(Domain::SP);
  simulation.setSolverType(Solver::Type::NRP);
  simulation.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simulation.doInitFromNodesAndTerminals(false);
  simulation.run();

  return loadNode->singleVoltage();
}

} // namespace

int main() {
  Logger::setLogDir("logs/" + simName);
  auto log = Logger::get(simName, Logger::Level::info, Logger::Level::info);

  Bool passed = true;

  auto freshLoad = SP::Ph1::Load::make("FreshLoad", Logger::Level::off);

  if (freshLoad->mPowerflowBusType != PowerflowBusType::PQ) {
    log->error("A load must report a PQ power flow bus type on construction.");
    passed = false;
  }

  const Complex defaulted =
      solveLoadBusVoltage(simName + "_DefaultBusType", false);
  const Complex explicitlyTyped =
      solveLoadBusVoltage(simName + "_ExplicitBusType", true);

  log->info("solved load bus voltage: default {:.6f} V at {:.6f} deg, "
            "explicit {:.6f} V at {:.6f} deg",
            std::abs(defaulted), std::arg(defaulted) * 180.0 / PI,
            std::abs(explicitlyTyped), std::arg(explicitlyTyped) * 180.0 / PI);

  if (std::abs(defaulted - explicitlyTyped) > 1e-9) {
    log->error("Leaving the load bus type at its default changed the power "
               "flow solution by {:.3e} V.",
               std::abs(defaulted - explicitlyTyped));
    passed = false;
  }

  if (std::abs(std::abs(defaulted) - nominalVoltage) < 1.0) {
    log->error("The load does not load the bus, so the comparison is void.");
    passed = false;
  }

  if (!passed) {
    log->error("Default power flow bus type check failed.");
    return 1;
  }

  log->info("Default power flow bus type check passed.");
  return 0;
}
