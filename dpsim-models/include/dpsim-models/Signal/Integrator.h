/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class Integrator : public SimSignalComp, public SharedFactory<Integrator> {

protected:
  /// Integration time step
  Real mTimeStep;

public:
  /// This is never explicitely set to reference anything, so the outside code is responsible for setting up the reference.
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

  Integrator(String name, Logger::Level logLevel = Logger::Level::off);
  /// Setter for integration step parameter
  void setParameters(Real timestep);
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
    PreStep(Integrator &integrator)
        : Task(**integrator.mName + ".PreStep"), mIntegrator(integrator) {
      mIntegrator.signalAddPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mIntegrator.signalPreStep(time, timeStepCount);
    };

  private:
    Integrator &mIntegrator;
  };

  class Step : public Task {
  public:
    Step(Integrator &integrator)
        : Task(**integrator.mName + ".Step"), mIntegrator(integrator) {
      mIntegrator.signalAddStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mIntegrator.signalStep(time, timeStepCount);
    };

  private:
    Integrator &mIntegrator;
  };
};
} // namespace Signal
} // namespace CPS
