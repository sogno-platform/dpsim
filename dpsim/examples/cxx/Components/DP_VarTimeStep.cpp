/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

// Checks updateTimeStep() and setEventRefinement() on DP networks: a change to
// the step already in use is a no-op, a coarse-to-fine change matches a run held
// fine, unconverted components are counted, a refined window recovers a
// switching transient, Ph3 converts, and a machine keeps its rotor angle.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "../Examples.h"
#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

struct Parameters {
  Real frequency = 50.0;
  Real sourceVoltage = 100e3;
  Real sourceResistance = 10.0;
  Real inductance = 50e-3;
  Real capacitance = 1e-6;
  Real loadResistance = 100.0;

  Real coarseStep = 200e-6;
  Real fineStep = 50e-6;
  Real changeTime = 0.1;
  Real finalTime = 0.2;

  Real machineStep = 1e-3;
  Real machineFineStep = 1e-4;
  Real machineChangeTime = 1.0;
  Real machineFinalTime = 1.2;

  Real switchResistance = 20.0;
  Real leadTime = 1e-3;
  Real followTime = 5e-3;
};

/// Source - R - L - probe, with C and a load resistor from probe to ground.
SystemTopology build(const Parameters &p,
                     std::shared_ptr<SimNode<Complex>> &probe) {
  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto res = DP::Ph1::Resistor::make("R_src", Logger::Level::off);
  res->setParameters(p.sourceResistance);

  auto ind = DP::Ph1::Inductor::make("L", Logger::Level::off);
  ind->setParameters(p.inductance);

  auto cap = DP::Ph1::Capacitor::make("C", Logger::Level::off);
  cap->setParameters(p.capacitance);

  auto load = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(p.loadResistance);

  vs->connect({SimNode<Complex>::GND, n1});
  res->connect({n1, n2});
  ind->connect({n2, SimNode<Complex>::GND});
  cap->connect({n2, SimNode<Complex>::GND});
  load->connect({n2, SimNode<Complex>::GND});

  probe = n2;

  return SystemTopology(p.frequency, SystemNodeList{n1, n2},
                        SystemComponentList{vs, res, ind, cap, load});
}

struct Trace {
  std::vector<Real> time;
  std::vector<Complex> voltage;
  UInt unconverted = 0;
};

/// changeTo <= 0 runs at startStep throughout.
Trace run(const Parameters &p, const String &name, Real startStep,
          Real changeTo) {
  const String simName = "DP_VarTimeStep_" + name;
  Logger::setLogDir("logs/" + simName);

  std::shared_ptr<SimNode<Complex>> probe;
  auto system = build(p, probe);

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(startStep);
  sim.setFinalTime(p.finalTime);
  sim.doSystemMatrixRecomputation(true);

  Trace trace;
  Bool changed = changeTo <= 0;

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    if (!changed && sim.time() >= p.changeTime - DOUBLE_EPSILON) {
      trace.unconverted = sim.updateTimeStep(changeTo);
      changed = true;
    }

    sim.next();
    trace.time.push_back(sim.time());
    trace.voltage.push_back(
        probe->attributeTyped<MatrixComp>("v")->get()(0, 0));
  }

  sim.stop();

  return trace;
}

/// Largest deviation between the samples the two traces share, relative to the
/// reference magnitude.
Real deviationAfter(const Trace &test, const Trace &reference, Real from) {
  Real worst = 0.0;
  Real scale = 0.0;

  for (const auto &value : reference.voltage)
    scale = std::max(scale, std::abs(value));

  for (std::size_t idx = 0; idx < test.time.size(); ++idx) {
    if (test.time[idx] < from)
      continue;

    for (std::size_t ref = 0; ref < reference.time.size(); ++ref) {
      if (std::abs(reference.time[ref] - test.time[idx]) > 1e-12)
        continue;
      worst =
          std::max(worst, std::abs(test.voltage[idx] - reference.voltage[ref]));
      break;
    }
  }

  return scale > 0 ? worst / scale : worst;
}

/// Source - extra - load, two steps with a change between them.
UInt unconvertedWith(const Parameters &p, const String &name,
                     SimPowerComp<Complex>::Ptr extra) {
  const String simName = "DP_VarTimeStep_" + name;
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto load = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(p.loadResistance);

  vs->connect({SimNode<Complex>::GND, n1});
  extra->connect({n1, n2});
  load->connect({n2, SimNode<Complex>::GND});

  auto system = SystemTopology(p.frequency, SystemNodeList{n1, n2},
                               SystemComponentList{vs, extra, load});

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.coarseStep);
  sim.setFinalTime(2.0 * p.coarseStep);
  sim.doSystemMatrixRecomputation(true);

  sim.start();
  sim.next();
  const UInt unconverted = sim.updateTimeStep(p.fineStep);
  sim.next();
  sim.stop();

  return unconverted;
}

/// SMIB with a 4th order VBR machine. Largest relative deviation of the terminal
/// voltage after the change, against a run held at the fine step.
Real machineDeviation(const Parameters &p) {
  using namespace CPS::CIM::Examples::Grids::SMIB::
      ReducedOrderSynchronGenerator;
  Scenario6::GridParams grid;
  CPS::CIM::Examples::Components::SynchronousGeneratorKundur::MachineParameters
      machine;

  // ----- power flow, for the machine's initial operating point -----
  Logger::setLogDir("logs/DP_VarTimeStep_MachinePF");

  auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);

  auto genPF = SP::Ph1::SynchronGenerator::make("gen", Logger::Level::off);
  genPF->setParameters(machine.nomPower, grid.VnomMV, grid.setPointActivePower,
                       grid.setPointVoltage, PowerflowBusType::PV);
  genPF->setBaseVoltage(grid.VnomMV);

  auto slackPF = SP::Ph1::NetworkInjection::make("slack", Logger::Level::off);
  slackPF->setParameters(grid.VnomMV);
  slackPF->setBaseVoltage(grid.VnomMV);
  slackPF->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto linePF = SP::Ph1::PiLine::make("line", Logger::Level::off);
  linePF->setParameters(grid.lineResistance, grid.lineInductance,
                        grid.lineCapacitance, grid.lineConductance);
  linePF->setBaseVoltage(grid.VnomMV);

  genPF->connect({n1PF});
  linePF->connect({n1PF, n2PF});
  slackPF->connect({n2PF});

  auto systemPF = SystemTopology(grid.nomFreq, SystemNodeList{n1PF, n2PF},
                                 SystemComponentList{genPF, linePF, slackPF});

  Simulation simPF("DP_VarTimeStep_MachinePF", Logger::Level::off);
  simPF.setSystem(systemPF);
  simPF.setTimeStep(0.1);
  simPF.setFinalTime(0.1);
  simPF.setDomain(Domain::SP);
  simPF.setSolverType(Solver::Type::NRP);
  simPF.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  simPF.doInitFromNodesAndTerminals(false);
  simPF.run();

  const Complex initPower(genPF->getApparentPower().real(),
                          genPF->getApparentPower().imag());
  const Real initMechPower = genPF->getApparentPower().real();
  const Complex initTerminal = n1PF->voltage()(0, 0);
  const Complex initSlack = n2PF->voltage()(0, 0);

  // ----- the two dynamic runs -----
  auto runMachine = [&](const String &name, Real baseStep,
                        Real changeTo) -> std::vector<std::pair<Real, Real>> {
    const String simName = "DP_VarTimeStep_" + name;
    Logger::setLogDir("logs/" + simName);

    auto n1 = SimNode<Complex>::make("n1", PhaseType::Single,
                                     std::vector<Complex>{initTerminal});
    auto n2 = SimNode<Complex>::make("n2", PhaseType::Single,
                                     std::vector<Complex>{initSlack});

    auto gen =
        DP::Ph1::SynchronGenerator4OrderVBR::make("gen", Logger::Level::off);
    gen->setOperationalParametersPerUnit(
        machine.nomPower, machine.nomVoltage, machine.nomFreq, machine.H,
        machine.Ld, machine.Lq, machine.Ll, machine.Ld_t, machine.Lq_t,
        machine.Td0_t, machine.Tq0_t);
    gen->setInitialValues(initPower, initMechPower, initTerminal);

    auto slack = DP::Ph1::NetworkInjection::make("slack", Logger::Level::off);
    slack->setParameters(grid.VnomMV);

    auto line = DP::Ph1::PiLine::make("line", Logger::Level::off);
    line->setParameters(grid.lineResistance, grid.lineInductance,
                        grid.lineCapacitance, grid.lineConductance);

    gen->connect({n1});
    line->connect({n1, n2});
    slack->connect({n2});

    auto system = SystemTopology(grid.nomFreq, SystemNodeList{n1, n2},
                                 SystemComponentList{gen, line, slack});

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(system);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(baseStep);
    sim.setFinalTime(p.machineFinalTime);
    sim.doSystemMatrixRecomputation(true);

    std::vector<std::pair<Real, Real>> trace;
    Bool changed = changeTo <= 0;

    sim.start();

    while (sim.time() < p.machineFinalTime + DOUBLE_EPSILON) {
      if (!changed && sim.time() >= p.machineChangeTime - DOUBLE_EPSILON) {
        sim.updateTimeStep(changeTo);
        changed = true;
      }

      sim.next();
      trace.emplace_back(
          sim.time(),
          std::abs(n1->attributeTyped<MatrixComp>("v")->get()(0, 0)));
    }

    sim.stop();

    return trace;
  };

  const auto fine = runMachine("MachineFine", p.machineFineStep, -1.0);
  const auto refined =
      runMachine("MachineRefined", p.machineStep, p.machineFineStep);

  Real scale = 0.0;
  for (const auto &sample : fine)
    scale = std::max(scale, sample.second);

  Real worst = 0.0;
  for (const auto &sample : refined) {
    if (sample.first < p.machineChangeTime)
      continue;
    for (const auto &reference : fine) {
      if (std::abs(reference.first - sample.first) > 1e-12)
        continue;
      worst = std::max(worst, std::abs(sample.second - reference.second));
      break;
    }
  }

  return scale > 0 ? worst / scale : worst;
}

/// Ph3 source, R, L, C and a PiLine, one step and a change.
UInt ph3Unconverted(const Parameters &p) {
  const String simName = "DP_VarTimeStep_Ph3";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::ABC);

  const Complex phase(p.sourceVoltage, 0.0);
  n1->setInitialVoltage(Math::singlePhaseVariableToThreePhase(phase));

  auto vs = DP::Ph3::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Math::singlePhaseVariableToThreePhase(phase), p.frequency);

  auto line = DP::Ph3::PiLine::make("line", Logger::Level::off);
  line->setParameters(
      Math::singlePhaseParameterToThreePhase(p.sourceResistance),
      Math::singlePhaseParameterToThreePhase(p.inductance),
      Math::singlePhaseParameterToThreePhase(p.capacitance));

  auto ind = DP::Ph3::Inductor::make("L", Logger::Level::off);
  ind->setParameters(Math::singlePhaseParameterToThreePhase(p.inductance));

  auto cap = DP::Ph3::Capacitor::make("C", Logger::Level::off);
  cap->setParameters(Math::singlePhaseParameterToThreePhase(p.capacitance));

  auto load = DP::Ph3::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(Math::singlePhaseParameterToThreePhase(p.loadResistance));

  vs->connect({SimNode<Complex>::GND, n1});
  line->connect({n1, n2});
  ind->connect({n2, n3});
  cap->connect({n3, SimNode<Complex>::GND});
  load->connect({n3, SimNode<Complex>::GND});

  auto system = SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                               SystemComponentList{vs, line, ind, cap, load});

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.coarseStep);
  sim.setFinalTime(2.0 * p.coarseStep);
  sim.doSystemMatrixRecomputation(true);

  sim.start();
  sim.next();
  const UInt unconverted = sim.updateTimeStep(p.fineStep);
  sim.next();
  sim.stop();

  return unconverted;
}

/// The load branch is interrupted at changeTime, which rings the L-C pair. With
/// refinement the fine step covers the event window only.
Real switchedPeak(const Parameters &p, const String &name, Real baseStep,
                  Real fineStep) {
  const String simName = "DP_VarTimeStep_" + name;
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto res = DP::Ph1::Resistor::make("R_src", Logger::Level::off);
  res->setParameters(p.sourceResistance);

  auto ind = DP::Ph1::Inductor::make("L", Logger::Level::off);
  ind->setParameters(p.inductance);

  auto cap = DP::Ph1::Capacitor::make("C", Logger::Level::off);
  cap->setParameters(p.capacitance);

  auto breaker = DP::Ph1::Switch::make("breaker", Logger::Level::off);
  breaker->setParameters(1e9, 1e-3, true);

  auto load = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(p.switchResistance);

  vs->connect({SimNode<Complex>::GND, n1});
  res->connect({n1, n2});
  ind->connect({n2, SimNode<Complex>::GND});
  cap->connect({n2, SimNode<Complex>::GND});
  breaker->connect({n2, n3});
  load->connect({n3, SimNode<Complex>::GND});

  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{vs, res, ind, cap, breaker, load});

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(baseStep);
  sim.setFinalTime(p.finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.addEvent(SwitchEvent::make(p.changeTime, breaker, false));

  if (fineStep > 0)
    sim.setEventRefinement(fineStep, p.leadTime, p.followTime);

  Real peak = 0.0;

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    sim.next();
    if (sim.time() >= p.changeTime && sim.time() <= p.changeTime + p.followTime)
      peak = std::max(
          peak, std::abs(n2->attributeTyped<MatrixComp>("v")->get()(0, 0)));
  }

  sim.stop();

  return peak;
}

} // namespace

int main(int argc, char *argv[]) {
  const Parameters p;

  const Trace coarse = run(p, "Coarse", p.coarseStep, -1.0);
  const Trace fine = run(p, "Fine", p.fineStep, -1.0);
  const Trace sameStep = run(p, "SameStep", p.coarseStep, p.coarseStep);
  const Trace refined = run(p, "Refined", p.coarseStep, p.fineStep);

  auto line = DP::Ph1::PiLine::make("line", Logger::Level::off);
  line->setParameters(p.sourceResistance, p.inductance, p.capacitance, 0.0);

  auto varSwitch = DP::Ph1::varResSwitch::make("switch", Logger::Level::off);
  varSwitch->setParameters(1e6, 1e-3, true);
  varSwitch->setInitParameters(p.coarseStep);

  auto ramp = DP::Ph1::VoltageSourceRamp::make("ramp", Logger::Level::off);
  ramp->setParameters(Complex(p.sourceVoltage, 0.0), Complex(0.0, 0.0),
                      p.frequency, 0.0, p.finalTime, 0.0);

  const Real window = p.changeTime + 1e-3;
  const Real sameDeviation = deviationAfter(sameStep, coarse, window);
  const Real refinedDeviation = deviationAfter(refined, fine, window);
  const UInt withLine = unconvertedWith(p, "PiLine", line);
  const UInt withSwitch = unconvertedWith(p, "VarResSwitch", varSwitch);
  const UInt withRamp = unconvertedWith(p, "RampSource", ramp);

  const Real peakFine = switchedPeak(p, "SwitchFine", p.fineStep, -1.0);
  const Real peakCoarse = switchedPeak(p, "SwitchCoarse", p.coarseStep, -1.0);
  const Real peakWindow =
      switchedPeak(p, "SwitchRefined", p.coarseStep, p.fineStep);

  const Real coarseError = std::abs(peakCoarse - peakFine) / peakFine;
  const Real windowError = std::abs(peakWindow - peakFine) / peakFine;

  const Bool passSame = sameDeviation < 1e-9;
  const Bool passRefined = refinedDeviation < 1e-2;
  const Bool passCount = sameStep.unconverted == 0 && withLine == 0 &&
                         withSwitch == 0 && withRamp == 1;
  const Bool passWindow = windowError < 0.5 * coarseError;
  const UInt ph3 = ph3Unconverted(p);
  const Bool passPh3 = ph3 == 0;
  const Real machine = machineDeviation(p);
  const Bool passMachine = machine < 1e-2;

  std::cout << std::scientific << std::setprecision(6);
  std::cout << "Same step is a no-op: " << (passSame ? "PASS" : "FAIL")
            << " (deviation " << sameDeviation << ")" << std::endl;
  std::cout << "Refined follows the fine run: "
            << (passRefined ? "PASS" : "FAIL") << " (deviation "
            << refinedDeviation << ")" << std::endl;
  std::cout << "Unconverted components counted: "
            << (passCount ? "PASS" : "FAIL") << " (passive "
            << sameStep.unconverted << ", PiLine " << withLine
            << ", varResSwitch " << withSwitch << ", ramped source " << withRamp
            << ")" << std::endl;

  std::cout << "Refined window recovers the transient: "
            << (passWindow ? "PASS" : "FAIL") << " (peak error, base step "
            << coarseError << ", refined " << windowError << ")" << std::endl;

  std::cout << "Ph3 network converts: " << (passPh3 ? "PASS" : "FAIL")
            << " (unconverted " << ph3 << ")" << std::endl;

  std::cout << "Machine follows the change: " << (passMachine ? "PASS" : "FAIL")
            << " (terminal voltage deviation " << machine << ")" << std::endl;

  const Bool passed = passSame && passRefined && passCount && passWindow &&
                      passPh3 && passMachine;
  std::cout << "Variable time step: " << (passed ? "6/6" : "failed")
            << std::endl;

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
