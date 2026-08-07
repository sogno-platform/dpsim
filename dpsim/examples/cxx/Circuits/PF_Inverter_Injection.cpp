// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cmath>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

const String simName = "PF_Inverter_Injection";

const Real nominalVoltage = 20e3;
const Real ratedApparentPower = 10e6;
const Real loadActivePower = 1e6;
const Real loadReactivePower = 0.5e6;
const Real generatorActivePower = 2e6;
const Real inverterSetPointActivePower = 1.5e6;
const Real inverterSetPointReactivePower = 0.3e6;

enum class InverterPosition { None, SlackBus, GeneratorBus, LoadBus };

struct Solution {
  Real slackActivePower = 0;
  Real slackReactivePower = 0;
  Real generatorReactivePower = 0;
  Real loadBusVoltage = 0;
};

Solution solve(const String &caseName, InverterPosition position,
               Real inverterActivePower, Real inverterReactivePower) {
  Logger::setLogDir("logs/" + caseName);

  auto slackNode = SimNode<Complex>::make("n1", PhaseType::Single);
  auto generatorNode = SimNode<Complex>::make("n2", PhaseType::Single);
  auto loadNode = SimNode<Complex>::make("n3", PhaseType::Single);

  auto slack = SP::Ph1::NetworkInjection::make("Slack", Logger::Level::off);
  slack->setParameters(nominalVoltage);
  slack->setBaseVoltage(nominalVoltage);
  slack->modifyPowerFlowBusType(PowerflowBusType::VD);
  slack->connect({slackNode});

  auto lineToGenerator =
      SP::Ph1::PiLine::make("PiLine_n1_n2", Logger::Level::off);
  lineToGenerator->setParameters(0.05, 0.1, 0);
  lineToGenerator->setBaseVoltage(nominalVoltage);
  lineToGenerator->connect({slackNode, generatorNode});

  auto lineToLoad = SP::Ph1::PiLine::make("PiLine_n2_n3", Logger::Level::off);
  lineToLoad->setParameters(0.05, 0.1, 0);
  lineToLoad->setBaseVoltage(nominalVoltage);
  lineToLoad->connect({generatorNode, loadNode});

  auto generator = SP::Ph1::SynchronGenerator::make("Gen", Logger::Level::off);
  generator->setParameters(ratedApparentPower, nominalVoltage,
                           generatorActivePower, nominalVoltage,
                           PowerflowBusType::PV);
  generator->setBaseVoltage(nominalVoltage);
  generator->modifyPowerFlowBusType(PowerflowBusType::PV);
  generator->connect({generatorNode});

  auto load = SP::Ph1::Load::make("Load", Logger::Level::off);
  load->setParameters(loadActivePower, loadReactivePower, nominalVoltage);
  load->modifyPowerFlowBusType(PowerflowBusType::PQ);
  load->connect({loadNode});

  SystemComponentList components{slack, lineToGenerator, lineToLoad, generator,
                                 load};

  auto inverter = SP::Ph1::AvVoltageSourceInverterDQ::make(
      "Inverter", "Inverter", Logger::Level::off, false);
  inverter->setParameters(2 * PI * 50, nominalVoltage, inverterActivePower,
                          inverterReactivePower);

  if (position != InverterPosition::None) {
    if (position == InverterPosition::SlackBus)
      inverter->connect({slackNode});
    else if (position == InverterPosition::GeneratorBus)
      inverter->connect({generatorNode});
    else
      inverter->connect({loadNode});
    components.push_back(inverter);
  }

  auto system =
      SystemTopology(50, SystemNodeList{slackNode, generatorNode, loadNode},
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
  solution.slackActivePower = **slack->mActivePowerInjection;
  solution.slackReactivePower = **slack->mReactivePowerInjection;
  solution.generatorReactivePower = **generator->mSetPointReactivePower;
  solution.loadBusVoltage = std::abs(loadNode->singleVoltage());
  return solution;
}

} // namespace

int main() {
  Logger::setLogDir("logs/" + simName);
  auto log = Logger::get(simName, Logger::Level::info, Logger::Level::info);

  Bool passed = true;
  const auto fail = [&passed, &log](const String &message) {
    log->error(message);
    passed = false;
  };

  const Solution without =
      solve(simName + "_NoInverter", InverterPosition::None, 0.0, 0.0);
  const Solution atSlack =
      solve(simName + "_AtSlackBus", InverterPosition::SlackBus,
            inverterSetPointActivePower, inverterSetPointReactivePower);
  const Solution atGenerator =
      solve(simName + "_AtGeneratorBus", InverterPosition::GeneratorBus, 0.0,
            inverterSetPointReactivePower);
  const Solution atLoad =
      solve(simName + "_AtLoadBus", InverterPosition::LoadBus,
            inverterSetPointActivePower, inverterSetPointReactivePower);

  log->info("slack power: no inverter {:.3f} W {:.3f} VAr, inverter on the "
            "slack bus {:.3f} W {:.3f} VAr",
            without.slackActivePower, without.slackReactivePower,
            atSlack.slackActivePower, atSlack.slackReactivePower);
  log->info("generator reactive power: no inverter {:.3f} VAr, reactive "
            "inverter on the generator bus {:.3f} VAr",
            without.generatorReactivePower, atGenerator.generatorReactivePower);

  const Real powerTolerance = 1e-6 * ratedApparentPower;
  const Real voltageTolerance = 1e-6 * nominalVoltage;

  if (std::abs(atSlack.loadBusVoltage - without.loadBusVoltage) >
      voltageTolerance)
    fail("An inverter on the slack bus must not move the solved voltages, so "
         "the reported slack power is the only thing left to compare.");

  if (std::abs(atSlack.slackActivePower - without.slackActivePower +
               inverterSetPointActivePower) > powerTolerance)
    fail("An inverter on the slack bus must reduce the reported slack active "
         "power by its active power set point.");

  if (std::abs(atSlack.slackReactivePower - without.slackReactivePower +
               inverterSetPointReactivePower) > powerTolerance)
    fail("An inverter on the slack bus must reduce the reported slack reactive "
         "power by its reactive power set point.");

  if (std::abs(atGenerator.loadBusVoltage - without.loadBusVoltage) >
      voltageTolerance)
    fail("A purely reactive inverter on a voltage controlled bus must not move "
         "the solved voltages, so the reported generator power is the only "
         "thing left to compare.");

  if (std::abs(atGenerator.generatorReactivePower -
               without.generatorReactivePower + inverterSetPointReactivePower) >
      powerTolerance)
    fail("An inverter on a generator bus must reduce the reactive power "
         "credited to the generator by its reactive power set point.");

  if (std::abs(atLoad.loadBusVoltage - without.loadBusVoltage) <
      voltageTolerance)
    fail("An inverter on a load bus must change the solved voltage, so the "
         "comparison against the other positions is void.");

  if (!passed) {
    log->error("Inverter injection check failed.");
    return 1;
  }

  log->info("Inverter injection check passed.");
  return 0;
}
