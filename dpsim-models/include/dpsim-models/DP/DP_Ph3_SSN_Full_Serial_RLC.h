// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph3_Capacitor.h>
#include <dpsim-models/Base/Base_Ph3_Inductor.h>
#include <dpsim-models/Base/Base_Ph3_Resistor.h>
#include <dpsim-models/DP/DP_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {
namespace SSN {

/// \brief Series RLC one-port as a three-phase DP V-type SSN component.
///
/// States x = [vC_abc; iL_abc], input = port voltage abc, output = inductor
/// current abc. R, L, C are 3x3 matrices, allowing phase coupling.
class Full_Serial_RLC final : public TwoTerminalVTypeSSNComp,
                              public SharedFactory<Full_Serial_RLC>,
                              public Base::Ph3::Resistor,
                              public Base::Ph3::Inductor,
                              public Base::Ph3::Capacitor {
public:
  using SharedFactory<Full_Serial_RLC>::make;

  Full_Serial_RLC(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off);
  Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Full_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void setParameters(const Matrix &resistance, const Matrix &inductance,
                     const Matrix &capacitance);
};

} // namespace SSN
} // namespace Ph3
} // namespace DP
} // namespace CPS
