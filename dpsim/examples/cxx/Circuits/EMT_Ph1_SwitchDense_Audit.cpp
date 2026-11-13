// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace DPsim;
using namespace CPS::EMT;

int main() {
  constexpr Real timeStep = 20e-6;
  constexpr Real finalTime = 2.0;
  constexpr Int switchCount = 32;

  auto sourceNode = SimNode::make("source", PhaseType::Single);
  auto loadNode = SimNode::make("load", PhaseType::Single);
  auto source = Ph1::VoltageSource::make("source");
  source->setParameters(Complex(100.0, 0.0), 50.0);
  auto load = Ph1::Resistor::make("load");
  load->setParameters(10.0);

  SystemComponentList components{source, load};
  SystemNodeList nodes{sourceNode};
  std::vector<SimNode::Ptr> switchNodes;
  switchNodes.reserve(switchCount);
  for (Int index = 0; index < switchCount - 1; ++index) {
    auto node =
        SimNode::make("switch_node" + std::to_string(index), PhaseType::Single);
    switchNodes.push_back(node);
    nodes.push_back(node);
  }
  nodes.push_back(loadNode);
  std::vector<std::shared_ptr<Ph1::Switch>> switches;
  switches.reserve(switchCount);
  for (Int index = 0; index < switchCount; ++index) {
    auto switchComp = Ph1::Switch::make("switch" + std::to_string(index));
    switchComp->setParameters(1e6, 0.1, false);
    const auto &fromNode = index == 0 ? sourceNode : switchNodes[index - 1];
    const auto &toNode =
        index + 1 == switchCount ? loadNode : switchNodes[index];
    switchComp->connect({fromNode, toNode});
    switches.push_back(switchComp);
    components.push_back(switchComp);
  }
  source->connect({SimNode::GND, sourceNode});
  load->connect({loadNode, SimNode::GND});

  auto system = SystemTopology(50.0, nodes, components);
  Simulation simulation("EMT_Ph1_SwitchDense_Audit", Logger::Level::off);
  simulation.setSystem(system);
  simulation.setDomain(Domain::EMT);
  simulation.setTimeStep(timeStep);
  simulation.setFinalTime(finalTime);
  simulation.doSystemMatrixRecomputation(true);
  for (const auto &switchComp : switches) {
    simulation.addEvent(SwitchEvent::make(0.5, switchComp, true));
    simulation.addEvent(SwitchEvent::make(1.5, switchComp, false));
  }

  const auto start = std::chrono::steady_clock::now();
  try {
    simulation.run();
  } catch (const CPS::SystemError &error) {
    std::cerr << "switch-dense simulation failed: " << error.descr() << '\n';
    return 2;
  } catch (const std::exception &error) {
    std::cerr << "switch-dense simulation failed: " << error.what() << '\n';
    return 2;
  }
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();

  const Real finalLoadVoltage = (**loadNode->attributeTyped<Matrix>("v"))(0, 0);
  const Bool allOpen =
      std::all_of(switches.begin(), switches.end(), [](const auto &switchComp) {
        return !(**switchComp->mIsClosed);
      });
  if (!allOpen || !std::isfinite(finalLoadVoltage) ||
      std::abs(finalLoadVoltage) > 1e-4) {
    std::cerr << "switch-dense regression: all_open=" << allOpen
              << ", load_voltage=" << finalLoadVoltage << '\n';
    return 1;
  }
  std::cout << "switch_count=" << switchCount << '\n'
            << "simulation_wall_s=" << elapsed << '\n';
  return 0;
}
