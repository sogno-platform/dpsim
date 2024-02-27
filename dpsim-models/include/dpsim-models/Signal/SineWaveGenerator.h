/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Signal/SignalGenerator.h>

namespace CPS {
namespace Signal {
/// \brief Model to generate sinusoidal signals
///
/// Inherits from the abstract SignalGenerator class.
/// The generated signal must be characterised by its initial complex phasor and its frequency.
class SineWaveGenerator : public SignalGenerator,
                          public SharedFactory<SineWaveGenerator> {
private:
  /// initial signal phasor phase
  Attribute<Real>::Ptr mPhase;
  Attribute<Real>::Ptr mMagnitude;

public:
  const Attribute<Complex>::Ptr mVoltageRef;
  /// init the identified object
  SineWaveGenerator(String name, Logger::Level logLevel = Logger::Level::off);
  /// set the source's parameters
  void setParameters(Complex initialPhasor, Real frequency = 0.0);
  /// implementation of inherited method step to update and return the current signal value
  void step(Real time);
};
} // namespace Signal
} // namespace CPS
