/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include "dpsim-models/Definitions.h"
#include "dpsim-models/EMT/EMT_Ph3_ControlledVoltageSource.h"
#include "dpsim-models/MathUtils.h"
#include <dpsim-models/EMT/EMT_Ph3_ControlledCurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph3_Resistor.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {

class DecouplingIdealTransformer_EMT_Ph3
    : public SimSignalComp,
      public SharedFactory<DecouplingIdealTransformer_EMT_Ph3> {
protected:
  Real mDelay;
  Matrix mInternalSeriesResistance =
      CPS::Math::singlePhaseParameterToThreePhase(1e-6);
  Matrix mInternalParallelResistance =
      CPS::Math::singlePhaseParameterToThreePhase(1e6);

  std::shared_ptr<EMT::SimNode> mNode1, mNode2, mVirtualNode;
  std::shared_ptr<EMT::Ph3::Resistor> mRes1, mRes2;
  std::shared_ptr<EMT::Ph3::ControlledCurrentSource> mCurrentSrc;
  std::shared_ptr<EMT::Ph3::ControlledVoltageSource> mVoltageSrc;
  Attribute<Matrix>::Ptr mSrcCurrent, mSrcVoltage;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  Matrix mCur1, mVol2;

  // Copy of the most recent elements of the ring buffers
  // They are used to perform extrapolation
  Matrix mCur1Extrap, mVol2Extrap;
  MatrixComp mCurrent1Extrap0;

  UInt mBufIdx = 0;
  UInt mMacroBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;
  CouplingMethod mCouplingMethod;
  UInt mExtrapolationDegree = 0;
  Matrix mVoltageSrcIntfCurr;

  Matrix interpolate(Matrix &data);
  Matrix extrapolate(Matrix &data);

public:
  typedef std::shared_ptr<DecouplingIdealTransformer_EMT_Ph3> Ptr;

  const Attribute<Matrix>::Ptr mSourceVoltageIntfVoltage;
  const Attribute<Matrix>::Ptr mSourceVoltageIntfCurrent;
  const Attribute<Matrix>::Ptr mSrcCurrentRef;
  const Attribute<Matrix>::Ptr mSrcVoltageRef;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingIdealTransformer_EMT_Ph3(
      String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Real>::Ptr node1, SimNode<Real>::Ptr node2,
                     Real delay, Matrix voltageSrcIntfCurr,
                     Matrix current1Extrap0,
                     CouplingMethod method = CouplingMethod::DELAY);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getComponents();
  TopologicalNode::Ptr getVirtualNode();

  class PreStep : public Task {
  public:
    PreStep(DecouplingIdealTransformer_EMT_Ph3 &itm)
        : Task(**itm.mName + ".MnaPreStep"), mITM(itm) {
      mPrevStepDependencies.push_back(mITM.mStates);
      mModifiedAttributes.push_back(mITM.mVoltageSrc->mVoltageRef);
      mModifiedAttributes.push_back(mITM.mCurrentSrc->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformer_EMT_Ph3 &mITM;
  };

  class PostStep : public Task {
  public:
    PostStep(DecouplingIdealTransformer_EMT_Ph3 &itm)
        : Task(**itm.mName + ".PostStep"), mITM(itm) {
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfCurrent);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfCurrent);
      mModifiedAttributes.push_back(mITM.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformer_EMT_Ph3 &mITM;
  };
};
} // namespace Signal
} // namespace CPS
