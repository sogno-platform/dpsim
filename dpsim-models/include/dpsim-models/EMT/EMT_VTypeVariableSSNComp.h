// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/EMT/EMT_VTypeSSNComp.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {

/// \brief Abstract base class for variable EMT V-type SSN components.
///
/// This class extends VTypeSSNComp with support for time-varying or
/// piecewise-linear state-space models whose MNA conductance stamp may change
/// during simulation. An affine output term
///
///   y = C x + D u + F
///
/// is supported in addition to the linear state-space model.
class VTypeVariableSSNComp : public VTypeSSNComp,
                             public MNAVariableCompInterface {
private:
  Bool mParameterChanged;
  Matrix mF;

protected:
  VTypeVariableSSNComp(String uid, String name, Int inputSize, Int outputSize,
                       Logger::Level logLevel = Logger::Level::off);

  Matrix calculateHistoryVector() const override final;
  void updateStateSpaceModel() override final;

  const Matrix &outputOffset() const;
  void setOutputOffset(const Matrix &F);

  /// Update the current stepwise-linearized component model.
  /// Returns true if the MNA conductance stamp changed.
  virtual Bool updateComponentParameters() = 0;

public:
  Bool hasParameterChanged() override final;

  void initializeFromNodesAndTerminals(Real frequency) override final;
};

} // namespace EMT
} // namespace CPS
