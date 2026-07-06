// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_ITypeSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// \brief Three-phase, two-terminal I-type SSN stamping layer.
///
/// Provides the concrete MNA stamps (Norton-equivalent 3x3 complex admittance
/// G = W^-1, history current, node voltage read) shared by three-phase
/// two-terminal I-type SSN components.
class TwoTerminalITypeSSNComp : public ITypeSSNComp {
protected:
  TwoTerminalITypeSSNComp(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override final;

public:
  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
};

} // namespace Ph3
} // namespace DP
} // namespace CPS
