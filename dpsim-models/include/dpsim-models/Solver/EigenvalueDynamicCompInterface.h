#pragma once
#include <dpsim-models/Solver/EigenvalueCompInterface.h>

namespace CPS {
template <typename VarType>
/// Interface to be implemented by all dynamic components taking part in eigenvalue extraction.
class EigenvalueDynamicCompInterface : public EigenvalueCompInterface {
public:
  using Ptr = std::shared_ptr<EigenvalueDynamicCompInterface>;
  using List = std::vector<Ptr>;

  /// Stamp component into sign matrix used for eigenvalue extraction.
  virtual void stampSignMatrix(UInt branchIdx, MatrixVar<VarType> &signMatrix,
                               Complex coeffDP) = 0;
  /// Stamp component into discretization matrix used for eigenvalue extraction.
  virtual void
  stampDiscretizationMatrix(UInt branchIdx,
                            MatrixVar<VarType> &discretizationMatrix,
                            Complex coeffDP) = 0;
};
} // namespace CPS