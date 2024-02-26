/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SP/SP_Ph1_ReducedOrderSynchronGeneratorVBR.h>

namespace CPS {
namespace SP {
namespace Ph1 {
/// @brief Voltage-Behind-Reactance (VBR) implementation
/// of 3rd order synchronous generator model
class SynchronGenerator3OrderVBR
    : public ReducedOrderSynchronGeneratorVBR,
      public SharedFactory<SynchronGenerator3OrderVBR> {
public:
  // #### Model specific variables ####
  /// voltage behind transient reactance
  const Attribute<Matrix>::Ptr mEdq_t;

protected:
  /// history term of VBR
  Matrix mEh_vbr;

public:
  ///
  SynchronGenerator3OrderVBR(const String &uid, const String &name,
                             Logger::Level logLevel = Logger::Level::off);
  ///
  SynchronGenerator3OrderVBR(const String &name,
                             Logger::Level logLevel = Logger::Level::off);

  // #### General Functions ####
  ///
  void specificInitialization() final;
  ///
  void stepInPerUnit() final;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
