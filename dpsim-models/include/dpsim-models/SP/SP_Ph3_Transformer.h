// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph3_Transformer.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/SP/SP_Ph3_Inductor.h>
#include <dpsim-models/SP/SP_Ph3_Resistor.h>

namespace CPS {
namespace SP {
namespace Ph3 {

class Transformer : public CompositePowerComp<Complex>,
                    public SharedFactory<Transformer>,
                    public Base::Ph3::Transformer {
private:
  std::shared_ptr<SP::Ph3::Resistor> mSubResistor;
  std::shared_ptr<SP::Ph3::Resistor> mSubResistor2;
  std::shared_ptr<SP::Ph3::Inductor> mSubInductor;
  std::shared_ptr<SP::Ph3::Inductor> mSubInductor2;
  std::shared_ptr<SP::Ph3::Resistor> mSubMagnetizingResistor;
  std::shared_ptr<SP::Ph3::Inductor> mSubMagnetizingInductor;

  /// Magnetizing resistance [Ohm]
  Matrix mMagnetizingResistance;
  /// Magnetizing inductance [H]
  Matrix mMagnetizingInductance;
  /// No-load current as a fraction of rated current (IEC 60076 i0)
  Real mNoLoadCurrent = 0.01;
  /// No-load loss as a fraction of rated power (IEC 60076 P0)
  Real mNoLoadLoss = 1e-3;
  /// True once the sub-components have been built
  Bool mSubCompCreated = false;
  /// Considers resistive losses with a series sub resistor
  Bool mWithResistiveLosses;

  /// Terminal index carrying the reference winding
  UInt mReferenceTerminal = 0;
  /// Terminal carrying the non-reference winding
  UInt nonReferenceTerminal() const { return 1 - mReferenceTerminal; }
  /// Turns ratio out of the reference winding
  Complex mRatioFromReference;
  /// Winding the nameplate impedance is referred to
  WindingReference mReferenceWinding = WindingReference::Auto;
  /// Resolves the reference winding
  void resolveWindingRoles();
  /// Builds the T-equivalent branches
  void createSubComponents() override;

public:
  /// Sets the magnetizing branch from the no-load test quantities
  void setMagnetizingBranch(Real noLoadCurrent, Real noLoadLoss) {
    mNoLoadCurrent = noLoadCurrent;
    mNoLoadLoss = noLoadLoss;
  }
  /// Selects the winding the nameplate impedance is referred to
  void setReferenceWinding(WindingReference referenceWinding) {
    mReferenceWinding = referenceWinding;
  }
  /// Defines UID, name and logging level
  Transformer(String uid, String name,
              Logger::Level logLevel = Logger::Level::off,
              Bool withResistiveLosses = false);
  /// Defines name and logging level
  Transformer(String name, Logger::Level logLevel = Logger::Level::off)
      : Transformer(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  /// Defines component parameters
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance);

  void initializeParentFromNodesAndTerminals(Real frequency) override;

  void mnaParentApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
};
} // namespace Ph3
} // namespace SP
} // namespace CPS
