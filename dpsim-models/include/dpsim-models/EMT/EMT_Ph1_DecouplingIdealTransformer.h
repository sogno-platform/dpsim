// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include "dpsim-models/Definitions.h"
#include "dpsim-models/EMT/EMT_Ph1_VoltageSource.h"
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/EMT/EMT_Ph1_CurrentSource.h>
#include <dpsim-models/EMT/EMT_Ph1_Resistor.h>

namespace CPS {
namespace EMT {
namespace Ph1 {

class DecouplingIdealTransformer
    : public CompositePowerComp<Real>,
      public SharedFactory<DecouplingIdealTransformer> {
protected:
  Real mDelay;
  Real mInternalSeriesResistance = 1e-6;
  Real mInternalParallelResistance = 1e6;

  std::shared_ptr<EMT::Ph1::Resistor> mRes1, mRes2;
  std::shared_ptr<EMT::Ph1::CurrentSource> mCurrentSrc;
  std::shared_ptr<EMT::Ph1::VoltageSource> mVoltageSrc;
  Attribute<Complex>::Ptr mSrcCurrent, mSrcVoltage;

  // Ringbuffers for the values of previous timesteps
  // TODO make these matrix attributes
  std::vector<Real> mCur1, mVol2;

  // Copy of the most recent elements of the ring buffers
  // They are used to perform extrapolation
  std::vector<Real> mCur1Extrap, mVol2Extrap;
  Real mCurrent1Extrap0;

  UInt mBufIdx = 0;
  UInt mMacroBufIdx = 0;
  UInt mBufSize;
  Real mAlpha;
  CouplingMethod mCouplingMethod;
  UInt mExtrapolationDegree = 0;
  Eigen::MatrixXd mVoltageSrcIntfCurr;

  // Get an approximate value of the signal in between steps when the delay is not an integer multiple of the step size
  Real interpolate(std::vector<Real> &data);

  // Estimates the value of the input signal in the next step
  Real extrapolate(std::vector<Real> &data);

public:
  typedef std::shared_ptr<DecouplingIdealTransformer> Ptr;

  const Attribute<Real>::Ptr mSourceVoltageIntfVoltage;
  const Attribute<Real>::Ptr mSourceVoltageIntfCurrent;
  const Attribute<Real>::Ptr mSrcCurrentRef;
  const Attribute<Real>::Ptr mSrcVoltageRef;

  ///FIXME: workaround for dependency analysis as long as the states aren't attributes
  const Attribute<Matrix>::Ptr mStates;

  DecouplingIdealTransformer(String uid, String name,
                             Logger::Level logLevel = Logger::Level::info);
  DecouplingIdealTransformer(String name,
                             Logger::Level logLevel = Logger::Level::info)
      : DecouplingIdealTransformer(name, name, logLevel) {}

  void setParameters(Real delay, Matrix voltageSrcIntfCurr,
                     Real current1Extrap0,
                     CouplingMethod method = CouplingMethod::DELAY);
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
} // namespace Ph1
} // namespace EMT
} // namespace CPS
