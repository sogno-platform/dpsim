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
namespace Ph1 {
class Transformer {
protected:
  /// Nominal voltage of primary side
  Real mNominalVoltagePrimary;
  /// Nominal voltage of secondary side
  Real mNominalVoltageSecondary;

public:
  /// Nominal voltage of the winding at the given terminal
  Real nominalVoltageAt(UInt terminal) const {
    return terminal == 0 ? mNominalVoltagePrimary : mNominalVoltageSecondary;
  }

  /// Rated Apparent Power [VA]
  const Attribute<Real>::Ptr mRatedPower;
  /// Complex transformer ratio
  const Attribute<Complex>::Ptr mRatio;
  /// Per-unit tap adjustment applied to the nominal complex transformer ratio
  const Attribute<Real>::Ptr mTapRatio;
  /// Resistance [Ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Inductance [H]
  const Attribute<Real>::Ptr mInductance;

  explicit Transformer(CPS::AttributeList::Ptr attributeList)
      : mRatedPower(attributeList->create<Real>("S")),
        mRatio(attributeList->create<Complex>("ratio")),
        mTapRatio(attributeList->create<Real>("tapRatio")),
        mResistance(attributeList->create<Real>("R")),
        mInductance(attributeList->create<Real>("L")) {
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
                     Real ratioAbs, Real ratioPhase, Real resistance,
                     Real inductance) {
    mNominalVoltagePrimary = nomVoltagePrimary;
    mNominalVoltageSecondary = nomVoltageSecondary;
    **mRatio = std::polar<Real>(ratioAbs, ratioPhase);
    **mResistance = resistance;
    **mInductance = inductance;
  }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
