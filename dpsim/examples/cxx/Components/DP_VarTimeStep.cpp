/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

// Checks Simulation::updateTimeStep() on a network of DP passive elements.
//
//   [1] Changing to the step already in use must reproduce the undisturbed run.
//       Any deviation is produced by the conversion itself.
//   [2] Changing from a coarse to a fine step must stay close to a run held at
//       the fine step throughout.
//   [3] The return value must count the components that did not convert.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <DPsim.h>

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

/// A PiLine is a CompositePowerComp whose parent hook does not convert.
UInt countUnconverted(const Parameters &p) {
  const String simName = "DP_VarTimeStep_Composite";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto line = DP::Ph1::PiLine::make("line", Logger::Level::off);
  line->setParameters(p.sourceResistance, p.inductance, p.capacitance, 0.0);

  auto load = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(p.loadResistance);

  vs->connect({SimNode<Complex>::GND, n1});
  line->connect({n1, n2});
  load->connect({n2, SimNode<Complex>::GND});

  auto system = SystemTopology(p.frequency, SystemNodeList{n1, n2},
                               SystemComponentList{vs, line, load});

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

} // namespace

int main(int argc, char *argv[]) {
  const Parameters p;

  const Trace coarse = run(p, "Coarse", p.coarseStep, -1.0);
  const Trace fine = run(p, "Fine", p.fineStep, -1.0);
  const Trace sameStep = run(p, "SameStep", p.coarseStep, p.coarseStep);
  const Trace refined = run(p, "Refined", p.coarseStep, p.fineStep);

  const Real window = p.changeTime + 1e-3;
  const Real sameDeviation = deviationAfter(sameStep, coarse, window);
  const Real refinedDeviation = deviationAfter(refined, fine, window);
  const UInt unconverted = countUnconverted(p);

  const Bool passSame = sameDeviation < 1e-9;
  const Bool passRefined = refinedDeviation < 1e-2;
  const Bool passCount = sameStep.unconverted == 0 && unconverted == 1;

  std::cout << std::scientific << std::setprecision(6);
  std::cout << "Same step is a no-op: " << (passSame ? "PASS" : "FAIL")
            << " (deviation " << sameDeviation << ")" << std::endl;
  std::cout << "Refined follows the fine run: "
            << (passRefined ? "PASS" : "FAIL") << " (deviation "
            << refinedDeviation << ")" << std::endl;
  std::cout << "Unconverted components counted: "
            << (passCount ? "PASS" : "FAIL") << " (passive "
            << sameStep.unconverted << ", with a PiLine " << unconverted << ")"
            << std::endl;

  const Bool passed = passSame && passRefined && passCount;
  std::cout << "Variable time step: " << (passed ? "3/3" : "failed")
            << std::endl;

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
