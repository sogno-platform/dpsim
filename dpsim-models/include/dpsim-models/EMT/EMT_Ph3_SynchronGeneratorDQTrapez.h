/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/EMT/EMT_Ph3_SynchronGeneratorDQ.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
class SynchronGeneratorDQTrapez
    : public SynchronGeneratorDQ,
      public SharedFactory<SynchronGeneratorDQTrapez> {
public:
  SynchronGeneratorDQTrapez(String uid, String name,
                            Logger::Level loglevel = Logger::Level::off);
  SynchronGeneratorDQTrapez(String name,
                            Logger::Level loglevel = Logger::Level::off);

  // #### MNA Section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;

  /// Add MNA pre step dependencies
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;

protected:
  // #### Trapezoidal Section ####

  /// Performs an Euler forward step with the state space model of a synchronous generator
  /// to calculate the flux and current from the voltage vector in per unit.
  void stepInPerUnit(Real time);
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
