/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>

#include <dpsim-models/Base/Base_Ph3_Inductor.h>
#include <dpsim-models/Solver/MNATearInterface.h>

namespace CPS {
namespace SP {
namespace Ph3 {
/// \brief Inductor
///
/// The inductor is represented by a DC equivalent circuit which corresponds to
/// one iteration of the trapezoidal integration method.
/// The equivalent DC circuit is a resistance in parallel with a current source.
/// The resistance is constant for a defined time step and system
/// frequency and the current source changes for each iteration.
class Inductor : public MNASimPowerComp<Complex>,
                 public Base::Ph3::Inductor,
                 public MNATearInterface,
                 public SharedFactory<Inductor> {
protected:
  /// susceptance [S]
  MatrixComp mSusceptance;

public:
  /// Defines UID, name, component parameters and logging level
  Inductor(String uid, String name,
           Logger::Level logLevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Inductor(String name, Logger::Level logLevel = Logger::Level::off)
      : Inductor(name, name, logLevel) {}
  /// Defines name, component parameters and logging level
  Inductor(String name, Real inductance,
           Logger::Level logLevel = Logger::Level::off);

  SimPowerComp<Complex>::Ptr clone(String name);

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency);
  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector);
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix);
  /// Upgrade values in the source vector and maybe system matrix before MNA solution
  void mnaStep(Matrix &systemMatrix, Matrix &rightVector, Matrix &leftVector,
               Real time);
  /// Upgrade internal variables after MNA solution
  void mnaMnaPostStep(Matrix &rightVector, Matrix &leftVector, Real time);
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector);
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector);

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  void mnaTearApplyMatrixStamp(SparseMatrixRow &tearMatrix);
};
} // namespace Ph3
} // namespace SP
} // namespace CPS
