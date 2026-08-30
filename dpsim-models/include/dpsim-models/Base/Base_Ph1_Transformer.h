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
  /// Resistance [Ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Inductance [H]
  const Attribute<Real>::Ptr mInductance;

  explicit Transformer(CPS::AttributeList::Ptr attributeList)
      : mRatedPower(attributeList->create<Real>("S")),
        mRatio(attributeList->create<Complex>("ratio")),
        mResistance(attributeList->create<Real>("R")),
        mInductance(attributeList->create<Real>("L")){};

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
