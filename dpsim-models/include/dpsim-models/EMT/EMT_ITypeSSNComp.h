// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_SSNComp.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for EMT I-type SSN components.
///
/// This class specializes SSNComp for components whose input is the interface
/// current and whose output is the interface voltage. It provides the common
/// logic for:
/// - mapping SSN input/output to interface current/voltage attributes
/// - steady-state initialization from a reconstructed input current
/// - voltage update from the SSN representation
/// - post-step state update.
class ITypeSSNComp : public SSNComp {
protected:
  ITypeSSNComp(String uid, String name, Int inputSize, Int outputSize,
               Logger::Level logLevel = Logger::Level::off);

  Attribute<Matrix>::Ptr inputAttribute() const override final;
  Attribute<Matrix>::Ptr outputAttribute() const override final;

  // Reconstruct the steady-state SSN input vector from node/terminal data.
  // For I-type SSN components, this corresponds to the interface current input.
  virtual MatrixComp buildInitialInputFromNodes(Real frequency) = 0;

public:
  void initializeFromNodesAndTerminals(Real frequency) override;

  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace EMT
} // namespace CPS
