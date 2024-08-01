/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Signal/CosineFMGenerator.h>
#include <dpsim-models/Signal/DCGenerator.h>
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/SignalGenerator.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Ideal Voltage source model
///
/// This component uses a SignalGenerator instance to produce an output signal on mIntfVoltage.
/// The signal generator can be configured to produce different signal shapes
/// By default or when the setParameters(Complex voltageRef, Real srcFreq = 0.0) function is used to configure this VoltageSource,
/// the SineWaveGenerator will be used. The frequency of the sine wave can be modified through the mSrcFreq attribute while the
/// magnitude and phase of the wave are derived from the magnitude and phase of the mVoltageRef attribute. Refer to the formula
/// in SineWaveGenerator.cpp for further details.
/// When one of the other setParameters functions is used to configure this VoltageSource, the output signal will not react to changes in
/// mVoltageRef or mSrcFreq. Instead, only the parameters given in the setParameters call are used to produce the signal.
///
/// This model uses modified nodal analysis to represent an ideal voltage source.
/// For a voltage source between nodes j and k, a new variable
/// (current across the voltage source) is added to the left side vector
/// as unkown and it is taken into account for the equation of node j as
/// positve and for the equation of node k as negative. Moreover
/// a new equation ej - ek = V is added to the problem.
class VoltageSource : public MNASimPowerComp<Complex>,
                      public DAEInterface,
                      public SharedFactory<VoltageSource> {
private:
  ///
  void updateVoltage(Real time);
  ///
  CPS::Signal::SignalGenerator::Ptr mSrcSig;

public:
  const CPS::Attribute<Complex>::Ptr mVoltageRef;
  const CPS::Attribute<Real>::Ptr mSrcFreq;

  /// Defines UID, name, component parameters and logging level
  VoltageSource(String uid, String name,
                Logger::Level loglevel = Logger::Level::off);
  /// Defines UID, name, component parameters and logging level
  VoltageSource(String name, Logger::Level logLevel = Logger::Level::off)
      : VoltageSource(name, name, logLevel) {}
  /// Defines name, component parameters and logging level
  VoltageSource(String name, Complex voltage,
                Logger::Level logLevel = Logger::Level::off);
  ///
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  ///
  void setSourceValue(Complex voltage);
  /// Setter for reference voltage and frequency with a sine wave generator
  /// This will initialize the values of mVoltageRef and mSrcFreq to match the given parameters
  /// However, the attributes can be modified during the simulation to dynamically change the magnitude, frequency, and phase of the sine wave.
  void setParameters(Complex voltageRef, Real srcFreq = 0.0);
  /// Setter for reference signal of type frequency ramp
  /// This will create a FrequencyRampGenerator which will not react to external changes to mVoltageRef or mSrcFreq!
  void setParameters(Complex initialPhasor, Real freqStart, Real rocof,
                     Real timeStart, Real duration, bool smoothRamp = true);
  /// Setter for reference signal of type cosine frequency modulation
  /// This will create a CosineFMGenerator which will not react to external changes to mVoltageRef or mSrcFreq!
  void setParameters(Complex initialPhasor, Real modulationFrequency,
                     Real modulationAmplitude, Real baseFrequency = 0.0,
                     bool zigzag = false);

  // #### MNA Section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompInitializeHarm(
      Real omega, Real timeStep,
      std::vector<Attribute<Matrix>::Ptr> leftVectors) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplySystemMatrixStampHarm(SparseMatrixRow &systemMatrix,
                                         Int freqIdx) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompApplyRightSideVectorStampHarm(Matrix &rightVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  class MnaPreStepHarm : public CPS::Task {
  public:
    MnaPreStepHarm(VoltageSource &voltageSource)
        : Task(**voltageSource.mName + ".MnaPreStepHarm"),
          mVoltageSource(voltageSource) {
      mAttributeDependencies.push_back(mVoltageSource.mVoltageRef);
      mModifiedAttributes.push_back(mVoltageSource.mRightVector);
      mModifiedAttributes.push_back(mVoltageSource.mIntfVoltage);
    }
    void execute(Real time, Int timeStepCount);

  private:
    VoltageSource &mVoltageSource;
  };

  class MnaPostStepHarm : public CPS::Task {
  public:
    MnaPostStepHarm(VoltageSource &voltageSource,
                    const std::vector<Attribute<Matrix>::Ptr> &leftVectors)
        : Task(**voltageSource.mName + ".MnaPostStepHarm"),
          mVoltageSource(voltageSource), mLeftVectors(leftVectors) {
      for (UInt i = 0; i < mLeftVectors.size(); i++)
        mAttributeDependencies.push_back(mLeftVectors[i]);
      mModifiedAttributes.push_back(mVoltageSource.mIntfCurrent);
    }
    void execute(Real time, Int timeStepCount);

  private:
    VoltageSource &mVoltageSource;
    std::vector<Attribute<Matrix>::Ptr> mLeftVectors;
  };

  // #### DAE Section ####
  /// Residual function for DAE Solver
  void daeResidual(double ttime, const double state[], const double dstate_dt[],
                   double resid[], std::vector<int> &off) override;
  ///Voltage Getter
  Complex daeInitialize() override;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
