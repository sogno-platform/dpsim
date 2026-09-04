/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Transformer.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// Transformer that includes an inductance and resistance
class Transformer : public CompositePowerComp<Complex>,
                    public SharedFactory<Transformer>,
                    public Base::Ph1::Transformer {
private:
  /// Internal resistor to model losses
  std::shared_ptr<DP::Ph1::Resistor> mSubResistor;
  /// Internal inductor to model losses
  std::shared_ptr<DP::Ph1::Inductor> mSubInductor;

  /// Second half of the leakage resistance, secondary-winding side
  std::shared_ptr<DP::Ph1::Resistor> mSubResistor2;
  /// Second half of the leakage inductance, secondary-winding side
  std::shared_ptr<DP::Ph1::Inductor> mSubInductor2;
  /// Core-loss resistance of the magnetizing branch
  std::shared_ptr<DP::Ph1::Resistor> mSubMagnetizingResistor;
  /// Magnetizing inductance of the magnetizing branch
  std::shared_ptr<DP::Ph1::Inductor> mSubMagnetizingInductor;

  /// Magnetizing resistance [Ohm]
  Real mMagnetizingResistance;
  /// Magnetizing inductance [H]
  Real mMagnetizingInductance;
  /// No-load current as a fraction of rated current (IEC 60076 i0)
  Real mNoLoadCurrent = 0.01;
  /// No-load loss as a fraction of rated power (IEC 60076 P0)
  Real mNoLoadLoss = 1e-3;

  /// Boolean for considering resistive losses with sub resistor
  Bool mWithResistiveLosses;

  /// Terminal carrying the reference winding
  UInt mReferenceTerminal = 0;
  /// Terminal carrying the non-reference winding
  UInt nonReferenceTerminal() const { return 1 - mReferenceTerminal; }
  /// Turns ratio out of the reference winding
  Complex mRatioFromReference;
  /// Sign carrying series quantities into the terminal 1 to 0 direction
  Real mOrientationSign = 1.;
  /// Winding the nameplate impedance is referred to
  WindingReference mReferenceWinding = WindingReference::Auto;
  /// Resolves the reference winding
  void resolveWindingRoles();
  /// True after createSubComponents() runs; prevents double-construction.

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

  // #### General ####
  /// Defines component parameters
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratioAbs, Real ratioPhase, Real resistance,
                     Real inductance);
  /// Set transformer specific parameters
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Real resistance, Real inductance);
  /// Constructs and registers MNA subcomponents (incl. terminal swap); idempotent.
  void createSubComponents() override;
  /// Initializes component from power flow data
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  /// Holds nothing derived from the timestep.
  bool mnaParentUpdateTimeStep(Real timeStep) override { return true; }
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
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
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
