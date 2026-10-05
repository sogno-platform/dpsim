/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cmath>
#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
namespace Base {
namespace Ph3 {
class Transformer {
protected:
  /// Nominal voltage of primary side
  Real mNominalVoltagePrimary;
  /// Nominal voltage of secondary side
  Real mNominalVoltageSecondary;
  /// Rated Apparent Power [VA]
  Real mRatedPower;
  /// Resistance [Ohm]
  Matrix mResistance;
  /// Inductance [H]
  Matrix mInductance;

public:
  /// Nominal voltage of the winding at the given terminal
  Real nominalVoltageAt(UInt terminal) const {
    return terminal == 0 ? mNominalVoltagePrimary : mNominalVoltageSecondary;
  }

  ///Transformer ratio
  const Attribute<Complex>::Ptr mRatio;
  /// Per-unit tap adjustment applied to the nominal complex transformer ratio
  const Attribute<Real>::Ptr mTapRatio;

  explicit Transformer(CPS::AttributeList::Ptr attributeList)
      : mRatio(attributeList->create<Complex>("ratio")),
        mTapRatio(attributeList->create<Real>("tapRatio")) {
    **mTapRatio = 1.;
  }

  /// Set a positive tap multiplier applied to the nominal complex ratio.
  void setTapRatio(Real tapRatio) {
    if (!std::isfinite(tapRatio) || tapRatio <= 0.)
      throw InvalidArgumentException();
    **mTapRatio = tapRatio;
  }

  Real tapRatio() const { return **mTapRatio; }

  Complex effectiveRatio() const { return **mRatio * **mTapRatio; }

  ///
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance) {
    mNominalVoltagePrimary = nomVoltagePrimary;
    mNominalVoltageSecondary = nomVoltageSecondary;
    mRatedPower = ratedPower;
    **mRatio = std::polar<Real>(ratioAbs, ratioPhase);
    mResistance = resistance;
    mInductance = inductance;
  }
};
} // namespace Ph3
} // namespace Base
} // namespace CPS
