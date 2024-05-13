/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_VoltageSource.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Resistive dynamic phasor voltage source
///
/// The real voltage source is a voltage source in series with a resistance,
/// which is transformed to a current source with
/// a parallel resistance using the Norton equivalent.
class VoltageSourceNorton : public MNASimPowerComp<Complex>,
                            public Base::Ph1::VoltageSource,
                            public SharedFactory<VoltageSourceNorton> {
protected:
  /// Equivalent current source [A]
  Complex mEquivCurrent;

  //  ### Real Voltage source parameters ###
  /// conductance [S]
  Real mConductance;

  /// Helper function for PreStep
  void updateState(Real time);

public:
  //  ### Real Voltage source parameters ###
  /// Resistance [ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Defines UID, name and logging level
  VoltageSourceNorton(String uid, String name,
                      Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  VoltageSourceNorton(String name, Logger::Level logLevel = Logger::Level::off)
      : VoltageSourceNorton(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  ///
  void setVoltageRef(Complex voltage) const;
  ///
  using Base::Ph1::VoltageSource::setParameters;
  ///
  /// THISISBAD: This declaration and Base::Ph1::VoltageSource::setParameters(Complex, Real) are ambiguous to each other. Clang does not like this.
  void setParameters(Complex voltage, Real srcFreq = -1, Real resistance = 1e9);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
