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
/// Models a ramp-like change in the signal's frequency with a given RoCoF.
/// The ramp is further characterised by start frequency, and the start time and duration of the event.
class FrequencyRampGenerator : public SignalGenerator,
                               public SharedFactory<FrequencyRampGenerator> {
private:
  /// magnitude of the initial signal phasor
  Real mMagnitude;
  /// phase of the initial signal phasor
  Real mInitialPhase;

  /// frequency before the event
  Real mFreqStart;
  /// frequency after the event
  Real mFreqEnd;
  /// event's rate of change of frequency
  Real mRocof;
  /// moment in time when the event starts
  Real mTimeStart;
  /// duration of the event
  Real mDuration;
  /// help variable for calculation
  Real mOldTime;

  /// boolean value to change the mode of calculation.
  /// If set to false, the current signal value will be calculated based on the previous.
  bool mUseAbsoluteCalc = true;
  /// to ensure a smooth frequency transition.
  /// If set to false, a linear frequency ramp with constant rocof and two kinks will be created.
  bool mSmoothRamp;

public:
  FrequencyRampGenerator(String name,
                         Logger::Level logLevel = Logger::Level::off)
      : SignalGenerator(name, logLevel) {
    SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
  }
  /// set frequency ramp specific parameters
  void setParameters(Complex initialPhasor, Real freqStart, Real ramp,
                     Real timeStart, Real duration, bool smoothRamp = true);
  /// implementation of inherited method step to update and return the current signal value
  void step(Real time);
  /// implementation of inherited method step to update and return the current signal value
  void stepAbsolute(Real time);
  /// update and return signal value using a cosine shaped ramp
  void stepSmooth(Real time);
};
} // namespace Signal
} // namespace CPS
