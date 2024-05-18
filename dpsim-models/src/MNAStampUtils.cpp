#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

void MNAStampUtils::stampConductance(Real conductance, SparseMatrixRow &mat,
                                     UInt node1Index, UInt node2Index,
                                     Bool isTerminal1NotGrounded,
                                     Bool isTerminal2NotGrounded) {
  stampValue(conductance, mat, node1Index, node2Index, isTerminal1NotGrounded,
             isTerminal2NotGrounded, 1, 0);
}

void MNAStampUtils::stampAdmittance(Complex admittance, SparseMatrixRow &mat,
                                    UInt node1Index, UInt node2Index,
                                    Bool isTerminal1NotGrounded,
                                    Bool isTerminal2NotGrounded, Int maxFreq,
                                    Int freqIdx) {
  stampValue(admittance, mat, node1Index, node2Index, isTerminal1NotGrounded,
             isTerminal2NotGrounded, maxFreq, freqIdx);
}

void MNAStampUtils::stampConductanceMatrix(const Matrix &conductanceMat,
                                           SparseMatrixRow &mat,
                                           UInt node1Index, UInt node2Index,
                                           Bool isTerminal1NotGrounded,
                                           Bool isTerminal2NotGrounded) {
  stampMatrix(conductanceMat, mat, node1Index, node2Index,
              isTerminal1NotGrounded, isTerminal2NotGrounded, 1, 0);
}

void MNAStampUtils::stampAdmittanceMatrix(const MatrixComp &admittanceMat,
                                          SparseMatrixRow &mat, UInt node1Index,
                                          UInt node2Index,
                                          Bool isTerminal1NotGrounded,
                                          Bool isTerminal2NotGrounded,
                                          Int maxFreq, Int freqIdx) {
  stampMatrix(admittanceMat, mat, node1Index, node2Index,
              isTerminal1NotGrounded, isTerminal2NotGrounded, maxFreq, freqIdx);
}

template <typename T>
void MNAStampUtils::stampValue(T value, SparseMatrixRow &mat, UInt node1Index,
                               UInt node2Index, Bool isTerminal1NotGrounded,
                               Bool isTerminal2NotGrounded, Int maxFreq,
                               Int freqIdx) {
  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    stampValueNoConditions(value, mat, node1Index, node2Index, maxFreq,
                           freqIdx);
  } else if (isTerminal1NotGrounded) {
    stampValueOnDiagonalNoConditions(value, mat, node1Index, maxFreq, freqIdx);
  } else if (isTerminal2NotGrounded) {
    stampValueOnDiagonalNoConditions(value, mat, node2Index, maxFreq, freqIdx);
  }
}

template <typename T>
void MNAStampUtils::stampMatrix(const MatrixVar<T> &matrix,
                                SparseMatrixRow &mat, UInt node1Index,
                                UInt node2Index, Bool isTerminal1NotGrounded,
                                Bool isTerminal2NotGrounded, Int maxFreq,
                                Int freqIdx) {
  Int numRows = matrix.rows();
  Int numCols = matrix.cols();
  if (numRows != numCols) {
    throw InvalidArgumentException();
  }

  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueNoConditions(matrix(i, j), mat, node1Index + i,
                               node2Index + j, maxFreq, freqIdx);
      }
    }
  } else if (isTerminal1NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueOnDiagonalNoConditions(matrix(i, j), mat, node1Index + i,
                                         maxFreq, freqIdx);
      }
    }
  } else if (isTerminal2NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueOnDiagonalNoConditions(matrix(i, j), mat, node2Index + j,
                                         maxFreq, freqIdx);
      }
    }
  }
}

template <typename T>
void MNAStampUtils::stampValueNoConditions(T value, SparseMatrixRow &mat,
                                           UInt node1Index, UInt node2Index,
                                           Int maxFreq, Int freqIdx) {
  stampValueOnDiagonalNoConditions(value, mat, node1Index, maxFreq, freqIdx);
  stampValueOnDiagonalNoConditions(value, mat, node2Index, maxFreq, freqIdx);
  stampValueOffDiagonalNoConditions(value, mat, node1Index, node2Index, maxFreq,
                                    freqIdx);
}

template <typename T>
void MNAStampUtils::stampValueOnDiagonalNoConditions(T value,
                                                     SparseMatrixRow &mat,
                                                     UInt nodeIndex,
                                                     Int maxFreq, Int freqIdx) {
  addToMatrixElement(mat, nodeIndex, nodeIndex, value, maxFreq, freqIdx);
}

template <typename T>
void MNAStampUtils::stampValueOffDiagonalNoConditions(
    T value, SparseMatrixRow &mat, UInt node1Index, UInt node2Index,
    Int maxFreq, Int freqIdx) {
  addToMatrixElement(mat, node1Index, node2Index, -value, maxFreq, freqIdx);
  addToMatrixElement(mat, node2Index, node1Index, -value, maxFreq, freqIdx);
}

// These wrapper functions standardize the signatures of "Math::addToMatrixElement" for Real and Complex "value" parameters,
// facilitating the use of templates in the stamping logic.
void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Real value,
                                       Int maxFreq, Int freqIdx) {
  Math::addToMatrixElement(mat, row, column, value);
}

void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Complex value,
                                       Int maxFreq, Int freqIdx) {
  Math::addToMatrixElement(mat, row, column, value, maxFreq, freqIdx);
}
