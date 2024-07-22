/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
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
/// \brief Ideal current source model
///
/// This model uses modified nodal analysis to represent an ideal current source.
/// This involves the stamping of the current to the right side vector.
class CurrentSource : public MNASimPowerComp<Real>,
                      public SharedFactory<CurrentSource> {
private:
  ///
  CPS::Signal::SignalGenerator::Ptr mSrcSig;

protected:
  // Updates current according to reference phasor and frequency
  void updateCurrent(Real time);

public:
  const Attribute<MatrixComp>::Ptr mCurrentRef;
  const Attribute<Real>::Ptr mSrcFreq;
  const Attribute<Complex>::Ptr mSigOut;

  /// Defines UID, name and logging level
  CurrentSource(String uid, String name,
                Logger::Level logLevel = Logger::Level::off);
  ///
  CurrentSource(String name, Logger::Level logLevel = Logger::Level::off)
      : CurrentSource(name, name, logLevel) {}

				SimPowerComp<Real>::Ptr clone(String name) override;
				// #### General ####
				/// Initializes component from power flow data
				void initializeFromNodesAndTerminals(Real frequency) override;
				/// Setter for reference Current
				void setParameters(MatrixComp currentRef, Real srcFreq = 50.0);

  // #### MNA section ####
  /// Initializes internal variables of the component
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Returns voltage through the component
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
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
