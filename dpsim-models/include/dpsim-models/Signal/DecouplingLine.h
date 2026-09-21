/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>

namespace CPS {
namespace Signal {
class DecouplingLine : public CompositePowerComp<Complex>,
                       public SharedFactory<DecouplingLine> {
protected:
  Real mDelay;
  Real mResistance;
  Real mInductance, mCapacitance;
  Real mSurgeImpedance;

  std::shared_ptr<DP::Ph1::Resistor> mRes1, mRes2;
  std::shared_ptr<DP::Ph1::CurrentSource> mSrc1, mSrc2;
  Attribute<Complex>::Ptr mSrcCur1, mSrcCur2;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Complex> mVolt1, mVolt2, mCur1, mCur2;
  UInt mBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;

  Complex interpolate(std::vector<Complex> &data);

public:
  typedef std::shared_ptr<DecouplingLine> Ptr;

  const Attribute<Complex>::Ptr mSrcCur1Ref;
  const Attribute<Complex>::Ptr mSrcCur2Ref;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingLine(String uid, String name,
                 Logger::Level logLevel = Logger::Level::info);
  DecouplingLine(String name, Logger::Level logLevel = Logger::Level::info)
      : DecouplingLine(name, name, logLevel) {}

  void setParameters(Real resistance, Real inductance, Real capacitance);
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
