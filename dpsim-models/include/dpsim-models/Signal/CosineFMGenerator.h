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
/// \brief Model to generate signals containing a frequency ramp
///
/// Inherits from the abstract SignalGenerator class.
/// Models a periodic frequency modulation of the signal's frequency.
/// The resulting signal oscillates inside a given frequency range.
class CosineFMGenerator : public SignalGenerator,
                          public SharedFactory<CosineFMGenerator> {
private:
  /// magnitude of the initial signal phasor
  Real mMagnitude;
  /// phase of the initial signal phasor
  Real mInitialPhase;
  /// signal's frequency
  Real mBaseFrequency;
  /// modulation frequency
  Real mModulationFrequency;
  /// amplitude of the modulation
  Real mModulationAmplitude;
  /// toggle a zig zag like frequency modulation
  bool mZigZag = false;

public:
  /// init the identified object
  CosineFMGenerator(String name, Logger::Level logLevel = Logger::Level::off)
      : SignalGenerator(name, logLevel), mZigZag(false) {
    SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
  }
  /// set the source's parameters
  void setParameters(Complex initialPhasor, Real modulationFrequency,
                     Real modulationAmplitude, Real frequency = 0.0,
                     bool zigzag = false);
  /// implementation of inherited method step to update and return the current signal value
  void step(Real time);
};
} // namespace Signal
} // namespace CPS
