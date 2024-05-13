/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Capacitor.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/PFSolverInterfaceBranch.h>

namespace CPS {
namespace SP {
namespace Ph1 {
class Capacitor : public MNASimPowerComp<Complex>,
                  public Base::Ph1::Capacitor,
                  public SharedFactory<Capacitor>,
                  public PFSolverInterfaceBranch {

protected:
  /// base apparent power[VA]
  Real mBaseApparentPower;
  /// base impedance [ohm]
  Real mBaseImpedance;
  /// base admittance [S]
  Real mBaseAdmittance;
  /// base voltage [V]
  Real mBaseVoltage;
  /// base current [A]
  Real mBaseCurrent;

  /// Impedance [pu]
  Complex mImpedancePerUnit;
  /// Admittance [pu]
  Complex mAdmittancePerUnit;

  /// Impedance [Ohm]
  Complex mImpedance;
  /// Admittance [S]
  Complex mAdmittance;
  /// Susceptance [S]
  Complex mSusceptance;

public:
  /// Defines UID, name and logging level
  Capacitor(String uid, String name,
            Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
      : Capacitor(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### Powerflow section ####
  /// Set base voltage
  void setBaseVoltage(Real baseVoltage);
  /// Initializes component from power flow data
  void calculatePerUnitParameters(Real baseApparentPower);
  /// Stamps admittance matrix
  void pfApplyAdmittanceMatrixStamp(SparseMatrixCompRow &Y) override;

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
