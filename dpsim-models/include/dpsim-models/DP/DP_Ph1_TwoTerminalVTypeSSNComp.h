// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_VTypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph1 {

/// \brief Single-phase, two-terminal V-type SSN stamping layer.
///
/// Provides the concrete MNA stamps (Norton admittance, history current, node
/// voltage read) shared by single-phase two-terminal V-type SSN components.
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

} // namespace Ph1
} // namespace DP
} // namespace CPS
