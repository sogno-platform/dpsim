// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace DC {

/// Scalar DC two-terminal V-type SSN port.
///
/// The interface voltage is v = v_terminal1 - v_terminal0. Positive interface
/// current flows from terminal 1 to terminal 0, so p = v * i follows the
/// passive sign convention.
class TwoTerminalVTypeSSNComp : public EMT::VTypeSSNComp {
protected:
  TwoTerminalVTypeSSNComp(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override final;
  void validateDCTerminals() const;

public:
  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
};

} // namespace DC
} // namespace EMT
} // namespace CPS
