// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph3_Capacitor.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalITypeSSNComp.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
namespace SSN {

class Capacitor final : public TwoTerminalITypeSSNComp,
                        public Base::Ph3::Capacitor,
                        public SharedFactory<Capacitor> {
public:
  using SharedFactory<Capacitor>::make;

  Capacitor(String uid, String name,
            Logger::Level logLevel = Logger::Level::off);
  Capacitor(String name, Logger::Level logLevel = Logger::Level::off)
      : Capacitor(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;

  void setParameters(Matrix capacitance);

protected:
  MatrixComp buildInitialInputFromNodes(Real frequency) override final;
};

} // namespace SSN
} // namespace Ph3
} // namespace EMT
} // namespace CPS
