// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_SSNComp.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for EMT V-type SSN components.
///
/// This class specializes SSNComp for components whose input is the interface
/// voltage and whose output is the interface current. It provides the common
/// logic for:
/// - mapping SSN input/output to interface voltage/current attributes
/// - steady-state initialization from a reconstructed input voltage
/// - current update from the Norton-form SSN representation
/// - post-step state update.
class VTypeSSNComp : public SSNComp {
protected:
  VTypeSSNComp(String uid, String name, Int inputSize, Int outputSize,
               Logger::Level logLevel = Logger::Level::off);

  Attribute<Matrix>::Ptr inputAttribute() const override final;
  Attribute<Matrix>::Ptr outputAttribute() const override final;

  // Reconstruct the steady-state SSN input vector from node/terminal data.
  // For V-type SSN components, this corresponds to the interface voltage input.
  virtual MatrixComp buildInitialInputFromNodes(Real frequency) = 0;

public:
  void initializeFromNodesAndTerminals(Real frequency) override;

  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace EMT
} // namespace CPS
