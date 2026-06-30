// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/DP/DP_SSNComp.h>

namespace CPS {
namespace DP {

/// \brief Abstract base for DP V-type SSN components (input voltage, output current).
class VTypeSSNComp : public SSNComp {
protected:
  VTypeSSNComp(String uid, String name, Int inputSize, Int outputSize,
               Logger::Level logLevel = Logger::Level::off);

  Attribute<MatrixComp>::Ptr inputAttribute() const override final;
  Attribute<MatrixComp>::Ptr outputAttribute() const override final;

  /// Reconstruct the initial input voltage from node/terminal phasors.
  virtual MatrixComp buildInitialInputFromNodes(Real frequency) = 0;

public:
  void initializeFromNodesAndTerminals(Real frequency) override;

  void mnaCompUpdateCurrent(const Matrix &leftVector) override final;

  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override final;
};

} // namespace DP
} // namespace CPS
