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
#include <dpsim-models/Signal/FrequencyRampGenerator.h>
#include <dpsim-models/Signal/SignalGenerator.h>
#include <dpsim-models/Signal/SineWaveGenerator.h>
#include <dpsim-models/Solver/DAEInterface.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/// \brief Ideal Voltage source model
///
/// This model uses modified nodal analysis to represent an ideal voltage source.
/// For a voltage source between nodes j and k, a new variable
/// (current across the voltage source) is added to the left side vector
/// as unkown and it is taken into account for the equation of node j as
/// positve and for the equation of node k as negative. Moreover
/// a new equation ej - ek = V is added to the problem.
class ControlledVoltageSource : public MNASimPowerComp<Complex>,
                                public SharedFactory<ControlledVoltageSource> {
private:
  ///
  void updateVoltage(Real time);

public:
  const Attribute<Complex>::Ptr mVoltageRef;
  const Attribute<Real>::Ptr mSrcFreq;

  /// Defines UID, name, component parameters and logging level
  ControlledVoltageSource(String uid, String name,
                          Logger::Level loglevel = Logger::Level::off);
  /// Defines UID, name, component parameters and logging level
  ControlledVoltageSource(String name,
                          Logger::Level logLevel = Logger::Level::off)
      : ControlledVoltageSource(name, name, logLevel) {}
  /// Defines name, component parameters and logging level
  ControlledVoltageSource(String name, Complex voltage,
                          Logger::Level logLevel = Logger::Level::off);
  ///
  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  /// Setter for reference voltage and frequency with a sine wave generator
  /// This will initialize the values of mVoltageRef and mSrcFreq to match the given parameters
  /// However, the attributes can be modified during the simulation to dynamically change the magnitude, frequency, and phase of the sine wave.
  void setParameters(Complex voltageRef);

  // #### MNA Section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
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
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
