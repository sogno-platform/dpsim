// SPDX-License-Identifier: MPL-2.0
#pragma once
#include <dpsim-models/Definitions.h>
#include <memory>
#include <vector>

namespace CPS {
/// Opt-in converged Newton coupling for variable-matrix EMT components.
/// Pre-step prepares the first tangent stamp. A check computes trial states
/// and true residuals without committing them. If any component fails its
/// check, every nonlinear component prepares a new tangent at that trial.
/// Post-step commits only after all checks pass. Histories of other components
/// remain fixed throughout the network iteration.
class MNANonlinearCompInterface {
public:
  using Ptr = std::shared_ptr<MNANonlinearCompInterface>;
  using List = std::vector<Ptr>;
  virtual ~MNANonlinearCompInterface() = default;
  virtual bool mnaCheckNonlinearConvergence(const Matrix &solution) = 0;
  virtual void mnaPrepareNonlinearIteration() = 0;
};
} // namespace CPS
