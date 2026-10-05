// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_VTypeSplitSSNComp.h>

namespace CPS {
namespace DP {
namespace Ph3 {

/// Common three-phase, two-terminal MNA coupling for split DP V-type SSN
/// components.
class TwoTerminalVTypeSplitSSNComp : public VTypeSplitSSNComp {
protected:
  TwoTerminalVTypeSplitSSNComp(String uid, String name, Int controllerStateSize,
                               Int controllerInputSize,
                               Int controllerOutputSize,
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
