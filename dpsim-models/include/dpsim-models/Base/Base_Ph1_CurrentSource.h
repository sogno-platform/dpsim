/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph1 {
class CurrentSource {
protected:
  /// Current set point [A]
  Complex mCurrentRef;
  /// Source frequency [Hz]
  Real mSrcFreq = -1;

public:
  /// Sets model specific parameters
  void setParameters(Complex currentRef, Real srcFreq = -1) {
    mCurrentRef = currentRef;
    mSrcFreq = srcFreq;
  }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
