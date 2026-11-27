// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>

namespace CPS::EMT::Ph3 {

/// Three-phase two-terminal V-type SSN base with independent terminal ports.
/// The input is [v0; v1] and the output is [i0; i1].
class MultiPortVTypeSSNComp : public VTypeSSNComp {
protected:
  MultiPortVTypeSSNComp(String uid, String name,
                        Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override final;

public:
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(RhsVector &rightVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
};

} // namespace CPS::EMT::Ph3
