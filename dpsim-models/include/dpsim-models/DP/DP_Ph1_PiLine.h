/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/Base/Base_Ph1_PiLine.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief PI-line dynamic phasor model
///
/// This model consists sub components to represent the
/// RLC elements of a PI-line.
class PiLine : public CompositePowerComp<Complex>,
               public MNATearInterface,
               public Base::Ph1::PiLine,
               public SharedFactory<PiLine> {
protected:
  Bool mUseSubcomponents = true;
  Real mDirectSeriesConductance = 0.;
  Real mDirectShuntConductance = 0.;
  Real mDirectShuntCapacitance = 0.;
  MatrixComp mDirectSeriesCond;
  MatrixComp mDirectInductorCond;
  MatrixComp mDirectSeriesPrevCurrentFactor;
  MatrixComp mDirectSeriesCurrent;
  MatrixComp mDirectSeriesVoltage;
  MatrixComp mDirectSeriesHistory;
  MatrixComp mDirectCapCond;
  MatrixComp mDirectCapPrevVoltageFactor;
  MatrixComp mDirectCapCurrent0;
  MatrixComp mDirectCapCurrent1;
  MatrixComp mDirectCapVoltage0;
  MatrixComp mDirectCapVoltage1;
  MatrixComp mDirectCapHistory0;
  MatrixComp mDirectCapHistory1;
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
  /// Parallel capacitor submodel at Terminal 1
  std::shared_ptr<Capacitor> mSubParallelCapacitor1;
  /// Right side vectors of subcomponents
  std::vector<const Matrix *> mRightVectorStamps;
  /// True after createSubComponents() runs; prevents double-construction.

public:
  /// Defines UID, name and logging level
  PiLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  PiLine(String uid, String name, Bool useSubcomponents,
         Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  PiLine(String name, Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, logLevel) {}
  PiLine(String name, Bool useSubcomponents,
         Logger::Level logLevel = Logger::Level::off)
      : PiLine(name, name, useSubcomponents, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String copySuffix) override;

  // #### General ####
  /// Constructs and registers MNA subcomponents; idempotent.
  void createSubComponents() override;
  /// Derives values from power flow data and pushes them to subcomponents
  void initializeParentFromNodesAndTerminals(Real frequency) override;
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  void mnaParentApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaParentApplyRightSideVectorStamp(Matrix &rightVector) override;

  // #### MNA section ####
  /// Updates internal current variable of the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates internal voltage variable of the component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre and post step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  /// add MNA pre and post step dependencies
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
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
} // namespace DP
} // namespace CPS
