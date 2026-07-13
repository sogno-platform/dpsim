// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class VCO : public SimSignalComp, public SharedFactory<VCO> {

protected:
  /// Nominal frequency (Input)
  Real mOmegaNom;
  /// Integration time step
  Real mTimeStep;

public:
  /// Input reference which comes as measurement from outside of the class
  const Attribute<Real>::Ptr mInputRef;
  /// Previous Input
  const Attribute<Real>::Ptr mInputPrev;
  /// Current Input
  const Attribute<Real>::Ptr mInputCurr;
  /// Previous State
  const Attribute<Real>::Ptr mStatePrev;
  /// Current State
  const Attribute<Real>::Ptr mStateCurr;
  /// Previous Output
  const Attribute<Real>::Ptr mOutputPrev;
  /// Current Output
  const Attribute<Real>::Ptr mOutputCurr;

  VCO(String name, Logger::Level logLevel = Logger::Level::off);
  /// Setter for VCO parameters
  void setParameters(Real omegaNom);
  /// Setter for simulation parameters
  void setSimulationParameters(Real timestep);
  /// Setter for initial values
  void setInitialValues(Real input_init, Real state_init, Real output_init);
  /// pre step operations
  void signalPreStep(Real time, Int timeStepCount);
  /// step operations
  void signalStep(Real time, Int timeStepCount);
  /// pre step dependencies
  void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                    AttributeBase::List &attributeDependencies,
                                    AttributeBase::List &modifiedAttributes);
  /// add step dependencies
  void signalAddStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes);

  Task::List getTasks();

  class PreStep : public Task {
  public:
    PreStep(VCO &VCO) : Task(**VCO.mName + ".PreStep"), mVCO(VCO) {
      mVCO.signalAddPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mVCO.signalPreStep(time, timeStepCount);
    };

  private:
    VCO &mVCO;
  };

  class Step : public Task {
  public:
    Step(VCO &VCO) : Task(**VCO.mName + ".Step"), mVCO(VCO) {
      mVCO.signalAddStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mVCO.signalStep(time, timeStepCount);
    };

  private:
    VCO &mVCO;
  };
};
} // namespace Signal
} // namespace CPS
