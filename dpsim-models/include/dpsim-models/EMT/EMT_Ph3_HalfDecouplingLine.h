/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/
#pragma once
#include <dpsim-models/Definitions.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// HalfDecouplingLine ....

class HalfDecouplingLine :
public CompositePowerComp<Real>,
public SharedFactory<HalfDecouplingLine> {
protected:

  Real mDelay;
  Matrix mResistance= Matrix::Zero(3, 3);
  Matrix mInductance= Matrix::Zero(3, 3);
  Matrix mCapacitance= Matrix::Zero(3, 3);
  Real mSurgeImpedance= 0;

  // ### Electrical Subcomponents ###
  /// Controlled current source
  std::shared_ptr<EMT::Ph3::ControlledCurrentSource> mSubCtrledCurrentSource;
  /// Parallel impedance calculated from line parameters
  std::shared_ptr<EMT::Ph3::Resistor> mSubRes;

  
  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  // For 3-Phase change vector to Matrix with time as row and phases as column
  // std::vector<Real> mVoltBuf, mCurBuf;
  // UInt mBufIdx = 0;
  // UInt mBufSize;
  // Real mAlpha;

  // Real interpolate(std::vector<Real> &data);

public:
  // typedef std::shared_ptr<HalfDecouplingLine> Ptr;

  const Attribute<Matrix>::Ptr mSrcCtrledCurrent;
  const Attribute<Matrix>::Ptr mReceivingVolt;
  const Attribute<Matrix>::Ptr mReceivingCur;
  const Attribute<Matrix>::Ptr mSendingVoltage;
  const Attribute<Matrix>::Ptr mSendingCur;

  /// Defines UID, name and logging level
  HalfDecouplingLine(String name, Logger::Level logLevel = Logger::Level::off) : HalfDecouplingLine(name, name, logLevel) {}
  HalfDecouplingLine(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  ///

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  /// Setter for reference current
  void setParameters(Matrix resistance, Matrix inductance, Matrix capacitance, 
                     Attribute<Matrix>::Ptr ReceivingVoltRef, Attribute<Matrix>::Ptr ReceivingCurRef);

  ///
  void Step(Real time);

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
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
