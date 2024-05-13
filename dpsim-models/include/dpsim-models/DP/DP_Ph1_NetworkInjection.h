/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_VoltageSource.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// \brief Network injection model
///
/// This model represents network injections by an ideal voltage source.
/// The voltage source can be configured to use different types of SignalGenerators using the various setParameters functions
/// When the SineWaveGenerator is configured via the void setParameters(Complex voltageRef, Real srcFreq = 0.0) function,
/// the frequency, magnitude and phase of the sine wave can be modified through the mVoltageRef and mSrcFreq attributes.
/// See DP_Ph1_VoltageSource.h for more details.
class NetworkInjection : public CompositePowerComp<Complex>,
                         public DAEInterface,
                         public SharedFactory<NetworkInjection> {
private:
  // ### Electrical Subcomponents ###
  /// Voltage source
  std::shared_ptr<DP::Ph1::VoltageSource> mSubVoltageSource;

  // #### solver ####
  /// Vector to collect subcomponent right vector stamps
  std::vector<const Matrix *> mRightVectorStamps;

public:
  const Attribute<Complex>::Ptr mVoltageRef;
  const Attribute<Real>::Ptr mSrcFreq;

  /// Defines UID, name and logging level
  NetworkInjection(String uid, String name,
                   Logger::Level loglevel = Logger::Level::off);
  /// Defines name and logging level
  NetworkInjection(String name, Logger::Level logLevel = Logger::Level::off)
      : NetworkInjection(name, name, logLevel) {}
  /// Defines name, component parameters and logging level
  NetworkInjection(String name, Complex voltage,
                   Logger::Level logLevel = Logger::Level::off);
  ///
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
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
  /// More General setter for voltage source parameters
  //void setVoltageSource(std::shared_ptr<DP::Ph1::VoltageSource> subVoltageSource);

  // #### MNA Section ####
  /// Stamps right side (source) vector
  void mnaParentApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Returns current through the component
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  /// Updates voltage across component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// MNA pre step operations
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  /// MNA post step operations
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA pre step dependencies
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  /// Add MNA post step dependencies
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

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
