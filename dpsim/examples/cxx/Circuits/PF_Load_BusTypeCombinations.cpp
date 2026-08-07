// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>
#include <vector>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

const String simName = "PF_Load_BusTypeCombinations";

const Real nominalVoltage = 20e3;
const Real ratedApparentPower = 10e6;
const Real loadActivePower = 1e6;
const Real loadReactivePower = 0.5e6;
const Real generatorActivePower = 2e6;
const Real lineResistance = 0.05;
const Real lineInductance = 0.1;

struct Placement {
  Bool load = false;
  Bool generator = false;
};

struct Solution {
  Real slackVoltage = 0;
  Real remoteVoltage = 0;
  Real slackActivePower = 0;
  Real slackReactivePower = 0;
};

Solution solve(const String &caseName, Placement atSlack, Placement atRemote) {
  Logger::setLogDir("logs/" + caseName);

  auto slackNode = SimNode<Complex>::make("n1", PhaseType::Single);
  auto remoteNode = SimNode<Complex>::make("n2", PhaseType::Single);

  auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  slack->setParameters(nominalVoltage);
  slack->setBaseVoltage(nominalVoltage);
  slack->modifyPowerFlowBusType(PowerflowBusType::VD);
  slack->connect({slackNode});

  auto line = SP::Ph1::PiLine::make("PiLine", Logger::Level::off);
  line->setParameters(lineResistance, lineInductance, 0);
  line->setBaseVoltage(nominalVoltage);
  line->connect({slackNode, remoteNode});

  SystemComponentList components{slack, line};

  const auto placeAt = [&](const Placement &placement, const String &suffix,
                           SimNode<Complex>::Ptr node) {
    if (placement.load) {
      auto load = SP::Ph1::Load::make("Load" + suffix, Logger::Level::off);
      load->setParameters(loadActivePower, loadReactivePower, nominalVoltage);
      load->connect({node});
      components.push_back(load);
    }

    if (placement.generator) {
      auto generator =
          SP::Ph1::SynchronGenerator::make("Gen" + suffix, Logger::Level::off);
      generator->setParameters(ratedApparentPower, nominalVoltage,
                               generatorActivePower, nominalVoltage,
                               PowerflowBusType::PV);
      generator->setBaseVoltage(nominalVoltage);
      generator->modifyPowerFlowBusType(PowerflowBusType::PV);
      generator->connect({node});
      components.push_back(generator);
    }
  };

  placeAt(atSlack, "Slack", slackNode);
  placeAt(atRemote, "Remote", remoteNode);

  auto system = SystemTopology(50, SystemNodeList{slackNode, remoteNode},
                               SystemComponentList{components});

  Simulation simulation(caseName, Logger::Level::off);
  simulation.setSystem(system);
  simulation.setTimeStep(1.0);
  simulation.setFinalTime(1.0);
  simulation.setDomain(Domain::SP);
  simulation.setSolverType(Solver::Type::NRP);
  simulation.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simulation.doInitFromNodesAndTerminals(false);
  simulation.run();

  Solution solution;
  solution.slackVoltage = std::abs(slackNode->singleVoltage());
  solution.remoteVoltage = std::abs(remoteNode->singleVoltage());
  solution.slackActivePower = **slack->mActivePowerInjection;
  solution.slackReactivePower = **slack->mReactivePowerInjection;
  return solution;
}

} // namespace

int main() {
  Logger::setLogDir("logs/" + simName);
  auto log = Logger::get(simName, Logger::Level::info, Logger::Level::info);

  Bool passed = true;
  const auto fail = [&passed](const String &message) {
    Logger::get(simName)->error(message);
    passed = false;
  };

  auto freshLoad = SP::Ph1::Load::make("FreshLoad", Logger::Level::off);
  if (freshLoad->mPowerflowBusType != PowerflowBusType::PQ)
    fail("A load must report a PQ power flow bus type on construction.");

  const Solution baseline = solve(simName + "_None_VD", {}, {});
  const Solution remoteLoad = solve(simName + "_PQ", {}, {true, false});
  const Solution remoteGenerator = solve(simName + "_PV", {}, {false, true});
  const Solution remoteBoth = solve(simName + "_PV_PQ", {}, {true, true});
  const Solution slackGenerator = solve(simName + "_VD_PV", {false, true}, {});
  const Solution slackLoad = solve(simName + "_VD_PQ", {true, false}, {});
  const Solution slackBoth = solve(simName + "_VD_PV_PQ", {true, true}, {});

  log->info("remote bus voltage: none {:.3f} V, load {:.3f} V, generator "
            "{:.3f} V, load and generator {:.3f} V",
            baseline.remoteVoltage, remoteLoad.remoteVoltage,
            remoteGenerator.remoteVoltage, remoteBoth.remoteVoltage);
  log->info("slack active power: alone {:.3f} W, with generator {:.3f} W, with "
            "load {:.3f} W, with both {:.3f} W",
            baseline.slackActivePower, slackGenerator.slackActivePower,
            slackLoad.slackActivePower, slackBoth.slackActivePower);

  const Real voltageTolerance = 1e-6 * nominalVoltage;
  const Real powerTolerance = 1e-6 * ratedApparentPower;

  if (std::abs(baseline.remoteVoltage - nominalVoltage) > voltageTolerance)
    fail("A node without components must carry no injection and stay at the "
         "slack voltage.");

  if (remoteLoad.remoteVoltage > nominalVoltage - voltageTolerance)
    fail("A node with only a load must be solved as a PQ bus, so its voltage "
         "must drop below the slack voltage.");

  if (std::abs(remoteGenerator.remoteVoltage - nominalVoltage) >
      voltageTolerance)
    fail("A node with only a generator must be solved as a PV bus, so its "
         "voltage must be held at the generator set point.");

  if (std::abs(remoteBoth.remoteVoltage - nominalVoltage) > voltageTolerance)
    fail("Adding a load to a node that holds a generator must leave it a PV "
         "bus, so its voltage must stay at the generator set point.");

  if (std::abs(remoteBoth.remoteVoltage - remoteLoad.remoteVoltage) <
      voltageTolerance)
    fail("Adding a generator to a load node did not change the solution, so "
         "the PQ to PV flip is not observable.");

  if (std::abs(slackLoad.slackVoltage - nominalVoltage) > voltageTolerance)
    fail("A load on the slack node must leave it a VD bus, so its voltage must "
         "stay at the slack set point.");

  if (std::abs(slackBoth.slackVoltage - nominalVoltage) > voltageTolerance)
    fail("A load and a generator on the slack node must leave it a VD bus, so "
         "its voltage must stay at the slack set point.");

  if (std::abs(slackLoad.slackActivePower - baseline.slackActivePower -
               loadActivePower) > powerTolerance)
    fail("The slack injection must cover a load connected to the slack node.");

  if (std::abs(slackLoad.slackReactivePower - baseline.slackReactivePower -
               loadReactivePower) > powerTolerance)
    fail("The slack injection must cover the reactive power of a load "
         "connected to the slack node.");

  if (std::abs(slackBoth.slackActivePower - slackGenerator.slackActivePower -
               loadActivePower) > powerTolerance)
    fail("The slack injection must cover a load sharing the slack node with a "
         "generator.");

  if (!passed) {
    log->error("Power flow bus type combination check failed.");
    return 1;
  }

  log->info("Power flow bus type combination check passed.");
  return 0;
}
