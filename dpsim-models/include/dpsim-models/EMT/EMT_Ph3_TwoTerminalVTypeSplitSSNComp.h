// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSplitSSNComp.h>

namespace CPS {
namespace EMT {
namespace Ph3 {

/// \brief Abstract base class for three-phase, two-terminal split V-type SSN
/// components.
///
/// This class implements the common three-phase/two-terminal logic:
/// - reconstruction of the initial input voltage from terminal phasors
/// - conductance matrix stamp
/// - right-side history current stamp
/// - interface voltage update from the MNA solution
///
/// It does not prescribe a particular controller or electrical plant.
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
} // namespace EMT
} // namespace CPS
