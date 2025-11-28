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
#include "dpsim-models/SP/SP_Ph1_ControlledVoltageSource.h"
#include "dpsim-models/SimNode.h"
#include "dpsim-models/TopologicalNode.h"
#include <dpsim-models/SP/SP_Ph1_ControlledCurrentSource.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class DecouplingIdealTransformer_SP_Ph1
    : public SimSignalComp,
      public SharedFactory<DecouplingIdealTransformer_SP_Ph1> {
protected:
  Real mDelay;
  Real mInternalSeriesResistance = 1e-6;
  Real mInternalParallelResistance = 1e6;

  std::shared_ptr<SP::SimNode> mNode1, mNode2, mVirtualNode;
  std::shared_ptr<SP::Ph1::Resistor> mRes1, mRes2;
  std::shared_ptr<SP::Ph1::ControlledCurrentSource> mCurrentSrc;
  std::shared_ptr<SP::Ph1::ControlledVoltageSource> mVoltageSrc;
  Attribute<Complex>::Ptr mSrcCurrent, mSrcVoltage;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Complex> mCur1, mVol2;

  // Copy of the most recent elements of the ring buffers
  // They are used to perform extrapolation
  std::vector<Complex> mCur1Extrap, mVol2Extrap;
  Complex mCurrent1Extrap0;

  UInt mBufIdx = 0;
  UInt mMacroBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;
  CouplingMethod mCouplingMethod;
  UInt mExtrapolationDegree = 0;
  Matrix mVoltageSrcIntfCurr;

  Complex interpolate(std::vector<Complex> &data);
  Complex extrapolate(std::vector<Complex> &data);

public:
  typedef std::shared_ptr<DecouplingIdealTransformer_SP_Ph1> Ptr;

  const Attribute<Complex>::Ptr mSourceVoltageIntfVoltage;
  const Attribute<Complex>::Ptr mSourceVoltageIntfCurrent;
  const Attribute<Complex>::Ptr mSrcCurrentRef;
  const Attribute<Complex>::Ptr mSrcVoltageRef;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingIdealTransformer_SP_Ph1(
      String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Complex>::Ptr node1, SimNode<Complex>::Ptr node2,
                     Real delay, Matrix voltageSrcIntfCurr,
                     Complex current1Extrap0,
                     CouplingMethod method = CouplingMethod::DELAY);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getComponents();
  TopologicalNode::Ptr getVirtualNode();

  class PreStep : public Task {
  public:
    PreStep(DecouplingIdealTransformer_SP_Ph1 &itm)
        : Task(**itm.mName + ".MnaPreStep"), mITM(itm) {
      mPrevStepDependencies.push_back(mITM.mStates);
      mModifiedAttributes.push_back(mITM.mVoltageSrc->mVoltageRef);
      mModifiedAttributes.push_back(mITM.mCurrentSrc->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformer_SP_Ph1 &mITM;
  };

  class PostStep : public Task {
  public:
    PostStep(DecouplingIdealTransformer_SP_Ph1 &itm)
        : Task(**itm.mName + ".PostStep"), mITM(itm) {
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfCurrent);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfCurrent);
      mModifiedAttributes.push_back(mITM.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformer_SP_Ph1 &mITM;
  };
};
} // namespace Signal
} // namespace CPS
