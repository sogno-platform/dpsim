// SPDX-FileCopyrightText: 2026 Institute for Automation of Power Systems, EONERC
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

#include <vector>

using namespace CPS;
using namespace DPsim;

int main() {
  constexpr UInt loadCount = 128;
  constexpr Real frequency = 50.0;
  constexpr Real timeStep = 50e-6;
  constexpr Real finalTime = 0.5;
  constexpr Real nominalVoltage = 400.0;
  constexpr Real totalActivePower = 30e3;
  constexpr Real totalReactivePower = 15e3;

  const String simName = "EMT_Ph3_PQLoad_GridBenchmark";
  Logger::setLogDir("logs/" + simName);

  const MatrixComp voltageNominal =
      Math::singlePhaseVariableToThreePhase(Complex(nominalVoltage, 0.0));
  auto loadBus = EMT::SimNode::make("load_bus", PhaseType::ABC);
  loadBus->setInitialVoltage(voltageNominal);

  auto source = EMT::Ph3::VoltageSource::make("source");
  source->setParameters(voltageNominal, frequency);
  source->connect({EMT::SimNode::GND, loadBus});

  std::vector<std::shared_ptr<EMT::Ph3::PQLoad>> loads;
  loads.reserve(loadCount);
  SystemComponentList components{source};
  for (UInt idx = 0; idx < loadCount; ++idx) {
    auto load = EMT::Ph3::PQLoad::make("load_" + std::to_string(idx));
    load->setParameters(totalActivePower / loadCount,
                        totalReactivePower / loadCount, nominalVoltage);
    load->connect({loadBus});
    components.push_back(load);
    loads.push_back(std::move(load));
  }

  auto system = SystemTopology(frequency, SystemNodeList{loadBus}, components);
  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_load_0", loads.front()->attribute("v_intf"));
  logger->logAttribute("i_load_0", loads.front()->attribute("i_intf"));
  logger->logAttribute("p_load_0", loads.front()->attribute("p_inst"));
  logger->logAttribute("q_load_0", loads.front()->attribute("q_inst"));
  logger->logAttribute("iterations_0", loads.front()->attribute("NIterations"));

  Simulation sim(simName);
  sim.setSystem(system);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  for (const auto &load : loads) {
    sim.addEvent(AttributeEvent<Real>::make(
        0.3, load->mActivePower, 2.0 * totalActivePower / loadCount));
    sim.addEvent(AttributeEvent<Real>::make(
        0.4, load->mReactivePower, 2.0 * totalReactivePower / loadCount));
  }

  sim.run();
}
