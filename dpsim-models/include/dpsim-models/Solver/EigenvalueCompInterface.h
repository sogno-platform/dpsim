#pragma once
#include <dpsim-models/Definitions.h>

namespace CPS {
/// Interface to be implemented by all components taking part in eigenvalue extraction.
class EigenvalueCompInterface {
public:
  using Ptr = std::shared_ptr<EigenvalueCompInterface>;
  using List = std::vector<Ptr>;

  /// Stamp component into branch<->node incidence matrix used for eigenvalue extraction.
  virtual void
  stampBranchNodeIncidenceMatrix(Matrix &branchNodeIncidenceMatrix) = 0;
  /// Set component's branch index.
  virtual void setBranchIdx(UInt i) = 0;
};
} // namespace CPS