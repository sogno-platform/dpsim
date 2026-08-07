/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include "dpsim-models/Definitions.h"
#include "dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h"
#include <vector>

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>

namespace CPS {
namespace Signal {
class DecouplingLineEMT_Ph3 : public CompositePowerComp<Real>,
                              public SharedFactory<DecouplingLineEMT_Ph3> {
protected:
  Real mDelay;
  Matrix mResistance = Matrix::Zero(3, 3);
  Matrix mInductance = Matrix::Zero(3, 3);
  Matrix mCapacitance = Matrix::Zero(3, 3);
  Matrix mSurgeImpedance;

  std::shared_ptr<EMT::Ph3::Resistor> mRes1, mRes2;
  std::shared_ptr<EMT::Ph3::ControlledCurrentSource> mSrc1, mSrc2;
  Attribute<Matrix>::Ptr mSrcCur1, mSrcCur2;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  Matrix mVolt1, mVolt2, mCur1, mCur2;
  UInt mBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;

  Matrix interpolate(Matrix &data);

public:
  typedef std::shared_ptr<DecouplingLineEMT_Ph3> Ptr;

  const Attribute<Matrix>::Ptr mSrcCur1Ref;
  const Attribute<Matrix>::Ptr mSrcCur2Ref;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingLineEMT_Ph3(String uid, String name,
                        Logger::Level logLevel = Logger::Level::info);
  DecouplingLineEMT_Ph3(String name,
                        Logger::Level logLevel = Logger::Level::info)
      : DecouplingLineEMT_Ph3(name, name, logLevel) {}

  void setParameters(Matrix resistance, Matrix inductance, Matrix capacitance);
  void step(Real time, Int timeStepCount);
  void postStep();

  // #### General ####
  void createSubComponents() override;
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // #### MNA section ####
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
};
} // namespace Signal
} // namespace CPS
