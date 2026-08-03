// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_DC_Capacitor.h>
#include <dpsim-models/EMT/EMT_DC_Inductor.h>
#include <dpsim-models/EMT/EMT_DC_Resistor.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Scalar DC lumped pi-line.
///
/// Positive line current flows from terminal 1 to terminal 0. Series R and L
/// connect the terminals through one virtual node. Half of the total shunt C
/// and G is connected from each terminal to ground.
class PiLine final : public CompositePowerComp<Real>,
                     public SharedFactory<PiLine> {
public:
  using SharedFactory<PiLine>::make;

  /// Series resistance [ohm].
  const Attribute<Real>::Ptr mSeriesResistance;
  /// Series inductance [H].
  const Attribute<Real>::Ptr mSeriesInductance;
  /// Total terminal-to-ground shunt capacitance [F].
  const Attribute<Real>::Ptr mParallelCapacitance;
  /// Total terminal-to-ground shunt conductance [S].
  const Attribute<Real>::Ptr mParallelConductance;
  /// Initial series current from terminal 1 to terminal 0 [A].
  const Attribute<Real>::Ptr mInitialCurrent;

  PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  PiLine(String name, Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;
  void setParameters(Real seriesResistance, Real seriesInductance,
                     Real parallelCapacitance = 0.0,
                     Real parallelConductance = 0.0, Real initialCurrent = 0.0);

  void createSubComponents() override final;
  void initializeParentFromNodesAndTerminals(Real frequency) override final;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaParentPreStep(Real time, Int timeStepCount) override final;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override final;
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override final;
  void mnaParentAddPostStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes,
      Attribute<Matrix>::Ptr &leftVector) override final;

private:
  std::shared_ptr<Resistor> mSeriesResistor;
  std::shared_ptr<Inductor> mSeriesInductor;
  std::shared_ptr<Resistor> mShuntResistor0;
  std::shared_ptr<Resistor> mShuntResistor1;
  std::shared_ptr<Capacitor> mShuntCapacitor0;
  std::shared_ptr<Capacitor> mShuntCapacitor1;

  void validateDCTerminals() const;
};

} // namespace DC
} // namespace EMT
} // namespace CPS
