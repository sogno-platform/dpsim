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
public:
  /// Nominal voltage of primary side
  const Attribute<Real>::Ptr mNominalVoltageEnd1;
  /// Nominal voltage of secondary side
  const Attribute<Real>::Ptr mNominalVoltageEnd2;
  /// Rated Apparent Power [VA]
  const Attribute<Real>::Ptr mRatedPower;
  /// Complex transformer ratio
  const Attribute<Complex>::Ptr mRatio;
  /// Resistance [Ohm]
  const Attribute<Real>::Ptr mResistance;
  /// Inductance [H]
  const Attribute<Real>::Ptr mInductance;
  /// Susceptance [Ohm]
  const Attribute<Real>::Ptr mCapacitance;
  /// Conductance [H]
  const Attribute<Real>::Ptr mConductance;

  explicit Transformer(CPS::AttributeList::Ptr attributeList)
      : mNominalVoltageEnd1(
            attributeList->create<Real>("nominal_voltage_end1")),
        mNominalVoltageEnd2(
            attributeList->create<Real>("nominal_voltage_end2")),
        mRatedPower(attributeList->create<Real>("S")),
        mRatio(attributeList->create<Complex>("ratio")),
        mResistance(attributeList->create<Real>("R")),
        mInductance(attributeList->create<Real>("L")),
        mCapacitance(attributeList->create<Real>("C")),
        mConductance(attributeList->create<Real>("G")){};

  ///
  void setParameters(Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratioAbs,
                     Real ratioPhase, Real resistance, Real inductance,
                     Real capacitance = 0, Real conductance = 0) {
    **mNominalVoltageEnd1 = nomVoltageEnd1;
    **mNominalVoltageEnd2 = nomVoltageEnd2;
    **mRatio = std::polar<Real>(ratioAbs, ratioPhase);
    **mResistance = resistance;
    **mInductance = inductance;
    **mCapacitance = capacitance;
    **mConductance = conductance;
  }
};
} // namespace Ph1
} // namespace Base
} // namespace CPS
