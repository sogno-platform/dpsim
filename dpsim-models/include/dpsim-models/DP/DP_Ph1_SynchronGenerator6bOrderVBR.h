/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/DP/DP_Ph1_ReducedOrderSynchronGeneratorVBR.h>

namespace CPS {
namespace DP {
namespace Ph1 {
/// @brief Voltage-Behind-Reactance (VBR) implementation
/// of 6th order synchronous generator model
/// Anderson - Fouad's model (Milano, Power System modelling and scripting, chapter 15)
class SynchronGenerator6bOrderVBR
    : public ReducedOrderSynchronGeneratorVBR,
      public SharedFactory<SynchronGenerator6bOrderVBR> {
public:
  // #### Model specific variables ####
  /// voltage behind transient reactance
  const Attribute<Matrix>::Ptr mEdq_t;
  /// voltage behind subtransient reactance
  const Attribute<Matrix>::Ptr mEdq_s;

protected:
  /// history term of voltage behind the transient reactance
  Matrix mEh_t;
  /// history term of voltage behind the subtransient reactance
  Matrix mEh_s;

public:
  ///
  SynchronGenerator6bOrderVBR(const String &uid, const String &name,
                              Logger::Level logLevel = Logger::Level::off);
  ///
  SynchronGenerator6bOrderVBR(const String &name,
                              Logger::Level logLevel = Logger::Level::off);

  // #### General Functions ####
  /// Initializes component from power flow data
  void specificInitialization() final;
  ///
  void stepInPerUnit() final;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
