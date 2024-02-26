/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/DP/DP_Ph1_CurrentSource.h>
#include <dpsim-models/DP/DP_Ph1_Resistor.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class DecouplingLine : public SimSignalComp,
                       public SharedFactory<DecouplingLine> {
protected:
  Real mDelay;
  Real mResistance;
  Real mInductance, mCapacitance;
  Real mSurgeImpedance;

  std::shared_ptr<DP::SimNode> mNode1, mNode2;
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

  DecouplingLine(String name, SimNode<Complex>::Ptr node1,
                 SimNode<Complex>::Ptr node2, Real resistance, Real inductance,
                 Real capacitance,
                 Logger::Level logLevel = Logger::Level::info);

  DecouplingLine(String name, Logger::Level logLevel = Logger::Level::info);

  void setParameters(SimNode<Complex>::Ptr node1, SimNode<Complex>::Ptr node2,
                     Real resistance, Real inductance, Real capacitance);
  void initialize(Real omega, Real timeStep);
  void step(Real time, Int timeStepCount);
  void postStep();
  Task::List getTasks();
  IdentifiedObject::List getLineComponents();

  class PreStep : public Task {
  public:
    PreStep(DecouplingLine &line)
        : Task(**line.mName + ".MnaPreStep"), mLine(line) {
      mPrevStepDependencies.push_back(mLine.mStates);
      mModifiedAttributes.push_back(mLine.mSrc1->mCurrentRef);
      mModifiedAttributes.push_back(mLine.mSrc2->mCurrentRef);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingLine &mLine;
  };

  class PostStep : public Task {
  public:
    PostStep(DecouplingLine &line)
        : Task(**line.mName + ".PostStep"), mLine(line) {
      mAttributeDependencies.push_back(mLine.mRes1->mIntfVoltage);
      mAttributeDependencies.push_back(mLine.mRes1->mIntfCurrent);
      mAttributeDependencies.push_back(mLine.mRes2->mIntfVoltage);
      mAttributeDependencies.push_back(mLine.mRes2->mIntfCurrent);
      mModifiedAttributes.push_back(mLine.mStates);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DecouplingLine &mLine;
  };
};
} // namespace Signal
} // namespace CPS
