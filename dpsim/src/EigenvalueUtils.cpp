#include <dpsim/EigenvalueUtils.h>

using namespace DPsim;

// TODO: consider resizing existing matrix instead of creating a new one (performance, memory)
MatrixComp EigenvalueUtils::returnNonZeroElements(const MatrixComp &mat) {
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask =
      (mat.array().abs() > 1e-14);
  int nonZeroCount = mask.cast<int>().sum();

  Eigen::MatrixXcd nonZeroMatrix(nonZeroCount, 1);
  int index = 0;
  for (int i = 0; i < mask.rows(); ++i) {
    for (int j = 0; j < mask.cols(); ++j) {
      if (mask(i, j)) {
        nonZeroMatrix(index++) = mat(i, j);
      }
    }
  }
  return nonZeroMatrix;
}

MatrixComp EigenvalueUtils::convertRealEquivalentToComplexMatrix(
    const Matrix &realEquivalentMatrix) {
  // The size of the complex matrix is half the size of the real matrix
  int size = realEquivalentMatrix.rows() / 2;

  // Create a complex matrix of the appropriate size
  MatrixComp complexMatrix(size, size);

  // Iterate over the complex matrix
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      // The real part is in the upper left quadrant of the real matrix
      double realPart = realEquivalentMatrix(i, j);

      // The imaginary part is in the lower left quadrant of the real matrix
      double imagPart = realEquivalentMatrix(i + size, j);

      // Assign the complex number to the complex matrix
      complexMatrix(i, j) = std::complex<double>(realPart, imagPart);
    }
  }
  return complexMatrix;
}
