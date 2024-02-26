/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph1 {
/// Static VAR compensator (SVC)
class SVC {
protected:
  /// Inductance [H]
  Real mInductance;
  /// Maximium susceptance [p.u.]
  Real mBMax;
  /// Minimium susceptance [p.u.]
  Real mBMin;
  /// rated B [S]
  Real mBN;
  /// maximum Q
  Real mQN;
  /// Time Constant
  Real mTr;
  /// Gain
  Real mKr;
  /// Reference Voltage
  Real mRefVolt;
  /// Nominal Voltage
  Real mNomVolt;
  // Time step values
  Real mPrevTimeStep = 0;
  Real mDeltaT;

  // param for mechanical model
  Bool mMechMode = false;
  Real mDeadband;
  Real mMechSwitchDelay;
  Real mTapPos;
  Real mMaxPos;
  Real mMinPos;

  // save for numerical integration
  Real mPrevVoltage;

public:
  Attribute<Real>::Ptr mDeltaV;
  Attribute<Real>::Ptr mBPrev;
  Attribute<Real>::Ptr mViolationCounter;

  /// Sets model specific parameters
  void setParameters(Real Bmax, Real Bmin, Real QN, Real nomVolt,
                     Real RefVolt = 0) {
    // initial inductance very high 10^6 [Ohm] @ 50 Hz
    mInductance = 3183.1;
    mBMax = Bmax;
    mBMin = Bmin;
    //mBMax = 1;
    //mBMin = Bmin;
    mQN = QN;
    mBN = QN / (nomVolt * nomVolt);

    mNomVolt = nomVolt;
    mRefVolt = (RefVolt > 0) ? RefVolt : mNomVolt;
  }

  void setControllerParameters(Real T, Real K) {
    // Pt1 controller
    mTr = T;
    mKr = K;
  }

  void setMechModelParameter(Real deadband, Real switchDelay, Real maxPos,
                             Real minPos, Real nomVolt, Real RefVolt, Real BN,
                             Real initPos = 0) {
    mMechMode = true;
    // initial inductance very high 10^6 [Ohm] @ 50 Hz
    mInductance = 3183.1;

    mDeadband = deadband;
    mMechSwitchDelay = switchDelay;
    mMaxPos = maxPos;
    mMinPos = minPos;
    mTapPos = initPos;

    mNomVolt = nomVolt;
    mRefVolt = (RefVolt > 0) ? RefVolt : mNomVolt;
    mBN = BN;
    mBMax = mBN;
    mBMin = mMinPos * mBN;
    mQN = mBN * nomVolt * nomVolt;
  }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
