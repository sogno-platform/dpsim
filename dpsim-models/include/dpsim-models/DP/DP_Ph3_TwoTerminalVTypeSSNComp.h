// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_VTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Three-phase, two-terminal V-type SSN stamping layer.
///
/// Provides the concrete MNA stamps (3x3 complex Norton admittance, history
/// current, node voltage read) shared by three-phase two-terminal V-type SSN
/// components. Each phase carries an independent complex envelope shifted by
/// the same single carrier frequency (see DP::SSNComp).
class TwoTerminalVTypeSSNComp : public VTypeSSNComp {
protected:
  TwoTerminalVTypeSSNComp(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override final;

public:
  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
