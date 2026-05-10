// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Ph3_Capacitor.h>
#include <dpsim-models/Base/Base_Ph3_Inductor.h>
#include <dpsim-models/Base/Base_Ph3_Resistor.h>
#include <dpsim-models/EMT/EMT_Ph3_TwoTerminalVTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
namespace SSN {

/// \brief Full_Serial_RLC
///
/// One-port circuit consisting of a resistor, an inductor and a capacitor
/// connected in series. The terminals are at the beginning and the end of the
/// component chain.
///
/// This class stores R, L and C parameters and constructs the corresponding
/// state-space matrices for a three-phase, two-terminal V-type SSN component:
///
///   x = [uC_abc; i_abc]
///   u = v_abc
///   y = i_abc
class Full_Serial_RLC final : public TwoTerminalVTypeSSNComp,
                              public SharedFactory<Full_Serial_RLC>,
                              public Base::Ph3::Resistor,
                              public Base::Ph3::Inductor,
                              public Base::Ph3::Capacitor {
public:
  using SharedFactory<Full_Serial_RLC>::make;

  /// Defines UID, name, component parameters and logging level
  Full_Serial_RLC(String uid, String name,
                  Logger::Level logLevel = Logger::Level::off);

  /// Defines name and logging level
  Full_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Full_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override final;

  void setParameters(const Matrix &resistance, const Matrix &inductance,
                     const Matrix &capacitance);
};

} // namespace SSN
} // namespace Ph3
} // namespace EMT
} // namespace CPS
