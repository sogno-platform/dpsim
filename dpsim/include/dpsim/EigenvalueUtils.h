#pragma once

#include <dpsim/Definitions.h>

namespace DPsim {
class EigenvalueUtils {
public:
  static MatrixComp returnNonZeroElements(const MatrixComp &mat);

  static MatrixComp
  convertRealEquivalentToComplexMatrix(const Matrix &realEquivalentMatrix);
};

} // namespace DPsim