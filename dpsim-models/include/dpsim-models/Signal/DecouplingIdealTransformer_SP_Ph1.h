// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include "dpsim-models/Definitions.h"
#include "dpsim-models/SP/SP_Ph1_ControlledVoltageSource.h"
#include "dpsim-models/SimNode.h"
#include "dpsim-models/TopologicalNode.h"
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/SP/SP_Ph1_ControlledCurrentSource.h>
#include <dpsim-models/SP/SP_Ph1_Resistor.h>

namespace CPS {
namespace Signal {
class DecouplingIdealTransformer_SP_Ph1
    : public CompositePowerComp<Complex>,
      public SharedFactory<DecouplingIdealTransformer_SP_Ph1> {
protected:
  Real mDelay;
  Real mInternalSeriesResistance = 1e-6;
  Real mInternalParallelResistance = 1e6;

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

  // Get an approximate value of the signal in between steps when the delay is not an integer multiple of the step size
  Complex interpolate(std::vector<Complex> &data);

  // Estimates the value of the input signal in the next step
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
      String uid, String name, Logger::Level logLevel = Logger::Level::info);
  DecouplingIdealTransformer_SP_Ph1(
      String name, Logger::Level logLevel = Logger::Level::info)
      : DecouplingIdealTransformer_SP_Ph1(name, name, logLevel) {}

  void setParameters(Real delay, Matrix voltageSrcIntfCurr,
                     Complex current1Extrap0,
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
} // namespace Signal
} // namespace CPS
