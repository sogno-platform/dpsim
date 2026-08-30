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
#include <dpsim-models/SP/SP_Ph1_Capacitor.h>
#include <dpsim-models/SP/SP_Ph1_Inductor.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/PFSolverInterfaceBranch.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/// Transformer that includes an inductance and resistance
class Transformer : public CompositePowerComp<Complex>,
                    public Base::Ph1::Transformer,
                    public SharedFactory<Transformer>,
                    public PFSolverInterfaceBranch {

private:
  /// True after createSubComponents() runs; prevents double-construction.
  /// Internal resistor to model losses
  std::shared_ptr<SP::Ph1::Resistor> mSubResistor;
  /// Internal inductor to model losses
  std::shared_ptr<SP::Ph1::Inductor> mSubInductor;

  /// Second half of the leakage resistance
  std::shared_ptr<SP::Ph1::Resistor> mSubResistor2;
  /// Second half of the leakage inductance
  std::shared_ptr<SP::Ph1::Inductor> mSubInductor2;
  /// Core-loss resistance of the magnetizing branch
  std::shared_ptr<SP::Ph1::Resistor> mSubMagnetizingResistor;
  /// Magnetizing inductance of the magnetizing branch
  std::shared_ptr<SP::Ph1::Inductor> mSubMagnetizingInductor;

  /// Transformer ratio magnitude
  Real mRatioAbs = 1;
  /// Transformer ratio pase [deg]
  Real mRatioPhase = 0;
  /// Nominal omega
  Real mNominalOmega;
  /// Reactance [Ohm]
  Real mReactance;

  /// Leakage
  Complex mLeakage;

  /// base apparent power[VA]
  Real mBaseApparentPower;
  /// base impedance [ohm]
  Real mBaseImpedance;
  /// base inductance [H]
  Real mBaseInductance;
  /// base admittance [S]
  Real mBaseAdmittance;
  ///base omega [1/s]
  Real mBaseOmega;

  ///base current [A]
  Real mBaseCurrent;

  /// resistance
  Real mResistancePerUnit;
  /// reactance
  Real mReactancePerUnit;
  /// inductance
  Real mInductancePerUnit;
  /// leakage impedance
  Complex mLeakagePerUnit;
  /// magnetizing impedance
  Complex mMagnetizingPerUnit;
  /// transformer ratio
  Real mRatioAbsPerUnit;
  /// complex per-unit turns ratio out of the reference winding
  Complex mRatioPerUnit;

  // #### Admittance matrix stamp ####
  MatrixComp mY_element;

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
  /// base voltage [V]
  const Attribute<Real>::Ptr mBaseVoltage;

  // #### Power flow results ####
  /// branch Current flow [A], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<MatrixComp>::Ptr mCurrent;

  /// branch active powerflow [W], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<Matrix>::Ptr mActivePowerBranch;

  /// branch reactive powerflow [Var], coef(0) has data from node 0, coef(1) from node 1.
  const Attribute<Matrix>::Ptr mReactivePowerBranch;
  /// nodal active power injection
  const Attribute<Real>::Ptr mActivePowerInjection;
  /// nodal reactive power injection
  const Attribute<Real>::Ptr mReactivePowerInjection;

  /// Defines UID, name and logging level
  Transformer(String uid, String name,
              Logger::Level logLevel = Logger::Level::off,
              Bool withResistiveLosses = false);
  /// Defines name and logging level
  Transformer(String name, Logger::Level logLevel = Logger::Level::off)
      : Transformer(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Set transformer specific parameters (without rated power)
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratioAbs, Real ratioPhase, Real resistance,
                     Real inductance);
  /// Set transformer specific parameters
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Real resistance, Real inductance);
  /// Constructs and registers MNA subcomponents; idempotent.
  void createSubComponents() override;
  /// Initializes component from power flow data
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // #### Powerflow section ####
  /// Get nominal voltage at end 1
  Real getNominalVoltagePrimary() const;
  /// Get nominal voltage at end 2
  Real getNominalVoltageSecondary() const;
  void setBaseVoltage(Real baseVoltage);
  /// Initializes component from power flow data
  void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
  /// Stamps admittance matrix
  void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) override;
  /// updates branch current and power flow, input pu value, update with real value
  void updateBranchFlow(VectorComp &current, VectorComp &powerflow);
  /// stores nodal injection power in this line object
  void storeNodalInjection(Complex powerInjection);

  // #### Getter ####
  /// get admittance matrix
  MatrixComp Y_element();

  // #### MNA Section ####
  /// Initializes internal variables of the component
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
} // namespace SP
} // namespace CPS
