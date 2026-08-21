// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <DPsim.h>

using namespace CPS;
using namespace DPsim;

int main() {
  const String simName = "EMT_Ph3_PQLoad";
  const Real frequency = 50.0;
  const Real timeStep = 50e-6;
  const Real finalTime = 0.5;

  const Real nominalVoltage = 400.0;        // line-to-line RMS [V]
  const Real activePower = 30e3;            // total three-phase power [W]
  const Real reactivePower = 15e3;          // total three-phase power [var]
  const Real activePowerAfterStep = 60e3;   // total three-phase power [W]
  const Real reactivePowerAfterStep = 30e3; // total three-phase power [var]

  Logger::setLogDir("logs/" + simName);

  const MatrixComp voltageNominal =
      Math::singlePhaseVariableToThreePhase(Complex(nominalVoltage, 0.0));
  const MatrixComp voltageLow =
      Math::singlePhaseVariableToThreePhase(Complex(320.0, 0.0));
  const MatrixComp voltageHigh =
      Math::singlePhaseVariableToThreePhase(Complex(440.0, 0.0));

  auto loadBus = EMT::SimNode::make("load_bus", PhaseType::ABC);
  loadBus->setInitialVoltage(voltageNominal);

  auto source = EMT::Ph3::VoltageSource::make("source");
  source->setParameters(voltageNominal, frequency);

  auto load = EMT::Ph3::PQLoad::make("load");
  load->setParameters(activePower, reactivePower, nominalVoltage);

  source->connect({EMT::SimNode::GND, loadBus});
  load->connect({loadBus});

  auto system = SystemTopology(frequency, SystemNodeList{loadBus},
                               SystemComponentList{source, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_load", load->attribute("v_intf"));
  logger->logAttribute("i_load", load->attribute("i_intf"));
  logger->logAttribute("p_reference", load->attribute("P"));
  logger->logAttribute("q_reference", load->attribute("Q"));
  logger->logAttribute("p_load", load->attribute("p_inst"));
  logger->logAttribute("q_load", load->attribute("q_inst"));
  logger->logAttribute("iterations", load->attribute("NIterations"));

  Simulation sim(simName);
  sim.setSystem(system);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  sim.addLogger(logger);

  sim.addEvent(
      AttributeEvent<MatrixComp>::make(0.1, source->mVoltageRef, voltageLow));
  sim.addEvent(
      AttributeEvent<MatrixComp>::make(0.2, source->mVoltageRef, voltageHigh));
  sim.addEvent(AttributeEvent<Real>::make(0.3, load->mActivePower,
                                          activePowerAfterStep));
  sim.addEvent(AttributeEvent<Real>::make(0.4, load->mReactivePower,
                                          reactivePowerAfterStep));

  sim.run();
}
