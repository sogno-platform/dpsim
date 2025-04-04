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
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
/// \brief Controlled Ideal Voltage source model
///
/// This model uses modified nodal analysis to represent an ideal voltage source.
/// This voltage source derives it's output purely from attributes rather than an internal signal generator.
class ControlledVoltageSource : public MNASimPowerComp<Real>,
                                public SharedFactory<ControlledVoltageSource> {
protected:
  // Updates voltage according to reference phasor and frequency
  void updateVoltage(Real time);

public:
  const CPS::Attribute<Matrix>::Ptr mVoltageRef;

  /// Defines UID, name and logging level
  ControlledVoltageSource(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);
  ///
  ControlledVoltageSource(String name,
                          Logger::Level logLevel = Logger::Level::off)
      : ControlledVoltageSource(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;
  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;
  /// Setter for reference voltage with a real valued generator
  /// This will initialize the values of mVoltageRef to match the given parameters
  /// However, the attributes can be modified during the simulation to dynamically change the value of the output voltage.
  void setParameters(Matrix voltageRef);

  // #### MNA section ####
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
} // namespace Ph3
} // namespace EMT
} // namespace CPS
