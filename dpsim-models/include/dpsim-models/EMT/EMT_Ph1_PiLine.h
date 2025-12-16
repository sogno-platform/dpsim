/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph1_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph1_Inductor.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// \brief PI-line dynamic phasor model
///
/// This model consists sub components to represent the
/// RLC elements of a PI-line.
class PiLine : public CompositePowerComp<Real>,
               public MNATearInterface,
               public Base::Ph1::PiLine,
               public SharedFactory<PiLine> {
protected:
  /// Series Inductance submodel
  std::shared_ptr<Inductor> mSubSeriesInductor;
  /// Series Resistor submodel
  std::shared_ptr<Resistor> mSubSeriesResistor;
  /// Parallel Resistor submodel at Terminal 0
  std::shared_ptr<Resistor> mSubParallelResistor0;
  // Parallel Capacitor submodel at Terminal 0
  std::shared_ptr<Capacitor> mSubParallelCapacitor0;
  /// Parallel resistor submodel at Terminal 1
  std::shared_ptr<Resistor> mSubParallelResistor1;
  // Parallel capacitor submodel at Terminal 1
  std::shared_ptr<Capacitor> mSubParallelCapacitor1;
  /// solver
  std::vector<const Matrix *> mRightVectorStamps;

public:
  /// Defines UID, name and logging level
  PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  PiLine(String name, Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String copySuffix) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Updates internal current variable of the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates internal voltage variable of the component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  MNAInterface::List mnaTearGroundComponents() override;
  void mnaTearInitialize(Real omega, Real timeStep) override;
  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix) override;
  void mnaTearApplyVoltageStamp(Matrix &voltageVector) override;
  void mnaTearPostStep(Complex voltage, Complex current) override;
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
