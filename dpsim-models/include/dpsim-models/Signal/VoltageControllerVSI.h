// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <vector>

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Task.h>

namespace CPS {
namespace Signal {
class VoltageControllerVSI : public SimSignalComp,
                             public SharedFactory<VoltageControllerVSI> {

protected:
  /// Simulation time step
  Real mTimeStep;

  // Voltage parameters
  Real mVdRef = 0;
  Real mVqRef = 0;

  // Voltage controller
  Real mKiVoltageCtrld;
  Real mKiVoltageCtrlq;
  Real mKpVoltageCtrld;
  Real mKpVoltageCtrlq;

  // Current controller
  Real mKiCurrCtrld;
  Real mKiCurrCtrlq;
  Real mKpCurrCtrld;
  Real mKpCurrCtrlq;

  /// initial values for states
  Real mPhi_dInit = 0;   //?
  Real mPhi_qInit = 0;   //?
  Real mGamma_dInit = 0; //?
  Real mGamma_qInit = 0; //?

  // state space matrices
  // check if dimension is right
  /// matrix A of state space model
  Matrix mA = Matrix::Zero(4, 4);
  /// matrix B of state space model
  Matrix mB = Matrix::Zero(4, 6);
  /// matrix C of state space model
  Matrix mC = Matrix::Zero(2, 4);
  /// matrix D of state space model
  Matrix mD = Matrix::Zero(2, 6);

public:
  // voltages and currents from filter in dq-frame
  const Attribute<Real>::Ptr mVc_d;
  const Attribute<Real>::Ptr mVc_q;
  const Attribute<Real>::Ptr mIrc_d;
  const Attribute<Real>::Ptr mIrc_q;

  // input, state and output vectors
  // matrixes of the seperate control loops previous and current
  /// Previous Input
  const Attribute<Matrix>::Ptr mInputPrev;
  /// Current Input
  const Attribute<Matrix>::Ptr mInputCurr;
  /// Previous State
  const Attribute<Matrix>::Ptr mStatePrev;
  /// Current State
  const Attribute<Matrix>::Ptr mStateCurr;
  /// Previous Output
  const Attribute<Matrix>::Ptr mOutputPrev;
  /// Current Output
  const Attribute<Matrix>::Ptr mOutputCurr;

  /// constructor
  VoltageControllerVSI(String name,
                       Logger::Level logLevel = Logger::Level::off);

  /// Setter for general parameters
  void setParameters(Real VampRef, Real OmegaRef);

  // Setter for VCO class
  void setControllerParameters(Real Kp_voltageCtrl, Real Ki_voltageCtrl,
                               Real Kp_currCtrl, Real Ki_currCtrl,
                               Real Omega_nominal);

  /// Setter for initial state values
  void setInitialStateValues(Real phi_dInit, Real phi_qInit, Real gamma_dInit,
                             Real gamma_qInit);

  /// Initialize vectors of state space model
  void initializeStateSpaceModel(Real omega, Real timeStep,
                                 Attribute<Matrix>::Ptr leftVector);

  /// pre step operations
  void signalPreStep(Real time, Int timeStepCount);
  /// pre step dependencies
  void signalAddPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                    AttributeBase::List &attributeDependencies,
                                    AttributeBase::List &modifiedAttributes);
  /// step operations
  void signalStep(Real time, Int timeStepCount);
  /// add step dependencies
  void signalAddStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes);

  Task::List getTasks();

  class PreStep : public Task {
  public:
    PreStep(VoltageControllerVSI &VoltageControllerVSI)
        : Task(**VoltageControllerVSI.mName + ".PreStep"),
          mVoltageControllerVSI(VoltageControllerVSI) {
      mVoltageControllerVSI.signalAddPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mVoltageControllerVSI.signalPreStep(time, timeStepCount);
    };

  private:
    VoltageControllerVSI &mVoltageControllerVSI;
  };
  class Step : public Task {
  public:
    Step(VoltageControllerVSI &VoltageControllerVSI)
        : Task(**VoltageControllerVSI.mName + ".Step"),
          mVoltageControllerVSI(VoltageControllerVSI) {
      mVoltageControllerVSI.signalAddStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mVoltageControllerVSI.signalStep(time, timeStepCount);
    };

  private:
    VoltageControllerVSI &mVoltageControllerVSI;
  };
};
} // namespace Signal
} // namespace CPS
