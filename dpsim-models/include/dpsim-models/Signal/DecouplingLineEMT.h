/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/EMT/EMT_Ph1_CurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class DecouplingLineEMT : public SimSignalComp,
                          public SharedFactory<DecouplingLineEMT> {
protected:
  Real mDelay;
  Real mResistance;
  Real mInductance;
  Real mCapacitance;
  Real mSurgeImpedance;

  std::shared_ptr<EMT::SimNode> mNode1, mNode2;
  std::shared_ptr<EMT::Ph1::Resistor> mRes1, mRes2;
  std::shared_ptr<EMT::Ph1::CurrentSource> mSrc1, mSrc2;
  Attribute<Complex>::Ptr mSrcCur1, mSrcCur2;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Real> mVolt1, mVolt2, mCur1, mCur2;
  UInt mBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;

  Real interpolate(std::vector<Real> &data);

public:
  typedef std::shared_ptr<DecouplingLineEMT> Ptr;

  const Attribute<Real>::Ptr mSrcCur1Ref;
  const Attribute<Real>::Ptr mSrcCur2Ref;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingLineEMT(String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Real>::Ptr node1, SimNode<Real>::Ptr node2,
                     Real resistance, Real inductance, Real capacitance);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getLineComponents();

  class PreStep : public Task {
  public:
    PreStep(DecouplingLineEMT &line)
        : Task(**line.mName + ".MnaPreStep"), mLine(line) {
      mPrevStepDependencies.push_back(mLine.mStates);
      mModifiedAttributes.push_back(mLine.mSrc1->mCurrentRef);
      mModifiedAttributes.push_back(mLine.mSrc2->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingLineEMT &mLine;
  };

  class PostStep : public Task {
  public:
    PostStep(DecouplingLineEMT &line)
        : Task(**line.mName + ".PostStep"), mLine(line) {
      mAttributeDependencies.push_back(mLine.mRes1->mIntfVoltage);
      mAttributeDependencies.push_back(mLine.mRes1->mIntfCurrent);
      mAttributeDependencies.push_back(mLine.mRes2->mIntfVoltage);
      mAttributeDependencies.push_back(mLine.mRes2->mIntfCurrent);
      mModifiedAttributes.push_back(mLine.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingLineEMT &mLine;
  };
};
} // namespace Signal
} // namespace CPS
