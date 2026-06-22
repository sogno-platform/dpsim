// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph1_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {
namespace SSN {

/// \brief Series RLC one-port as a DP V-type SSN component.
///
/// States x = [v_C; i_L], input = port voltage, output = inductor current.
class Full_Serial_RLC final : public TwoTerminalVTypeSSNComp,
                              public SharedFactory<Full_Serial_RLC> {
public:
  using SharedFactory<Full_Serial_RLC>::make;

  Full_Serial_RLC(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off);
  Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Full_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void setParameters(Real resistance, Real inductance, Real capacitance);

private:
  Real mResistance = 0.;
  Real mInductance = 0.;
  Real mCapacitance = 0.;
};

} // namespace SSN
} // namespace Ph1
} // namespace DP
} // namespace CPS
