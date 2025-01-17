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
#include "dpsim-models/EMT/EMT_Ph1_VoltageSource.h"
#include <dpsim-models/EMT/EMT_Ph1_CurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {

enum CouplingMethod {
    DELAY,
    EXTRAPOLATION_ZOH,
    EXTRAPOLATION_LINEAR
};

class DecouplingIdealTransformerEMT : public SimSignalComp,
                          public SharedFactory<DecouplingIdealTransformerEMT> {
protected:
  Real mDelay;

  std::shared_ptr<EMT::SimNode> mNode1, mNode2;
  std::shared_ptr<EMT::Ph1::CurrentSource> mCurrentSrc;
  std::shared_ptr<EMT::Ph1::VoltageSource> mVoltageSrc;
  Attribute<Complex>::Ptr mSrcCurrent, mSrcVoltage;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Real> mCur1, mVol2;

  // Copy of the most recent elements of the ring buffers
  // They are used to perform extrapolation
  std::vector<Real> mCur1Extrap, mVol2Extrap;

  UInt mBufIdx = 0;
  UInt mMacroBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;
  CouplingMethod mCouplingMethod;
  UInt mExtrapolationDegree = 0;
  Eigen::MatrixXd mVoltageSrcIntfCurr;

  Real interpolate(std::vector<Real> &data);
  Real extrapolate(std::vector<Real> &data);

public:
  typedef std::shared_ptr<DecouplingIdealTransformerEMT> Ptr;

  const Attribute<Real>::Ptr mSrcCurrentRef;
  const Attribute<Real>::Ptr mSrcVoltageRef;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingIdealTransformerEMT(String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Real>::Ptr node1, SimNode<Real>::Ptr node2, Real delay, Eigen::MatrixXd voltageSrcIntfCurr, CouplingMethod method);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getComponents();

  class PreStep : public Task {
  public:
    PreStep(DecouplingIdealTransformerEMT &itm)
        : Task(**itm.mName + ".MnaPreStep"), mITM(itm) {
      mPrevStepDependencies.push_back(mITM.mStates);
      mModifiedAttributes.push_back(mITM.mVoltageSrc->mVoltageRef);
      mModifiedAttributes.push_back(mITM.mCurrentSrc->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformerEMT &mITM;
  };

  class PostStep : public Task {
  public:
    PostStep(DecouplingIdealTransformerEMT &itm)
        : Task(**itm.mName + ".PostStep"), mITM(itm) {
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mVoltageSrc->mIntfCurrent);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfVoltage);
      mAttributeDependencies.push_back(mITM.mCurrentSrc->mIntfCurrent);
      mModifiedAttributes.push_back(mITM.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingIdealTransformerEMT &mITM;
  };
};
} // namespace Signal
} // namespace CPS
