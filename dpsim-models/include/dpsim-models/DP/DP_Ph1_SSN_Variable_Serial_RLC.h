// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_Ph1_MixedVTypeVariableSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {
namespace SSN {

/// Series RLC one-port on MixedVTypeVariableSSNComp; R/L/C held constant.
/// States x = [v_C; i_L], input = port voltage, output = inductor current.
class Variable_Serial_RLC final : public MixedVTypeVariableSSNComp,
                                  public SharedFactory<Variable_Serial_RLC> {
public:
  using SharedFactory<Variable_Serial_RLC>::make;

  Variable_Serial_RLC(String uid, String name,
                      Logger::Level logLevel = Logger::Level::off);
  Variable_Serial_RLC(String name, Logger::Level logLevel = Logger::Level::off)
      : Variable_Serial_RLC(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void setParameters(Real resistance, Real inductance, Real capacitance,
                     Real omegaN);

protected:
  Bool updateComponentParameters() override;

private:
  Real mResistance = 0.;
  Real mInductance = 0.;
  Real mCapacitance = 0.;
  Real mOmegaN = 0.;

  void buildStateSpaceModel(Matrix &A, Matrix &B, Matrix &C, Matrix &D) const;
};

} // namespace SSN
} // namespace Ph1
} // namespace DP
} // namespace CPS
