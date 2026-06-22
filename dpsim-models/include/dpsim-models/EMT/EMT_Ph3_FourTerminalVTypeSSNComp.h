// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

class FourTerminalVTypeSSNComp : public EMT::VTypeSSNComp {
protected:
  Matrix mTerminalIncidence;

  FourTerminalVTypeSSNComp(String uid, String name,
                           Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override;

public:
  const Matrix &terminalIncidenceMatrix() const;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;

  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
