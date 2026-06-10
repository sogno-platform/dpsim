// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_ITypeSSNComp.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Abstract base class for three-phase, two-terminal, I-type SSN components.
///
/// This class implements the common three-phase/two-terminal logic:
/// - zero initial current input
/// - current source right-side vector stamp
/// - interface voltage update from the MNA solution
class TwoTerminalITypeSSNComp : public ITypeSSNComp {
protected:
  TwoTerminalITypeSSNComp(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);

  MatrixComp buildInitialInputFromNodes(Real frequency) override;

public:
  void
  mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override final;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override final;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override final;
};

} // namespace Ph3
} // namespace EMT
} // namespace CPS
