#include <dpsim-models/MNAStampUtils.h>

using namespace CPS;

void MNAStampUtils::stampConductance(Real conductance, SparseMatrixRow &mat,
                                     UInt node1Index, UInt node2Index,
                                     Bool isTerminal1NotGrounded,
                                     Bool isTerminal2NotGrounded,
                                     const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Start stamping conductance...");

  stampValue(conductance, mat, node1Index, node2Index, isTerminal1NotGrounded,
             isTerminal2NotGrounded, 1, 0, mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampAdmittance(Complex admittance, SparseMatrixRow &mat,
                                    UInt node1Index, UInt node2Index,
                                    Bool isTerminal1NotGrounded,
                                    Bool isTerminal2NotGrounded,
                                    const Logger::Log &mSLog, Int maxFreq,
                                    Int freqIdx) {
  SPDLOG_LOGGER_DEBUG(
      mSLog, "Start stamping admittance for frequency index {:d}...",
      freqIdx);

  stampValue(admittance, mat, node1Index, node2Index, isTerminal1NotGrounded,
             isTerminal2NotGrounded, maxFreq, freqIdx, mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampConductanceMatrix(const Matrix &conductanceMat,
                                           SparseMatrixRow &mat,
                                           UInt node1Index, UInt node2Index,
                                           Bool isTerminal1NotGrounded,
                                           Bool isTerminal2NotGrounded,
                                           const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog, "Start stamping conductance matrix...");

  stampMatrix(conductanceMat, mat, node1Index, node2Index,
              isTerminal1NotGrounded, isTerminal2NotGrounded, 1, 0, mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampAdmittanceMatrix(
    const MatrixComp &admittanceMat, SparseMatrixRow &mat, UInt node1Index,
    UInt node2Index, Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
    const Logger::Log &mSLog, Int maxFreq, Int freqIdx) {
  SPDLOG_LOGGER_DEBUG(
      mSLog,
      "Start stamping admittance matrix for frequency index {:d}...",
      freqIdx);

  stampMatrix(admittanceMat, mat, node1Index, node2Index,
              isTerminal1NotGrounded, isTerminal2NotGrounded, maxFreq, freqIdx,
              mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

template <typename T>
void MNAStampUtils::stampValue(T value, SparseMatrixRow &mat, UInt node1Index,
                               UInt node2Index, Bool isTerminal1NotGrounded,
                               Bool isTerminal2NotGrounded, Int maxFreq,
                               Int freqIdx, const Logger::Log &mSLog) {
  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    stampValueNoConditions(value, mat, node1Index, node2Index, maxFreq, freqIdx,
                           mSLog);
  } else if (isTerminal1NotGrounded) {
    stampValueOnDiagonalNoConditions(value, mat, node1Index, maxFreq, freqIdx,
                                     mSLog);
  } else if (isTerminal2NotGrounded) {
    stampValueOnDiagonalNoConditions(value, mat, node2Index, maxFreq, freqIdx,
                                     mSLog);
  }
}

template <typename T>
void MNAStampUtils::stampMatrix(const MatrixVar<T> &matrix,
                                SparseMatrixRow &mat, UInt node1Index,
                                UInt node2Index, Bool isTerminal1NotGrounded,
                                Bool isTerminal2NotGrounded, Int maxFreq,
                                Int freqIdx, const Logger::Log &mSLog) {
  Int numRows = matrix.rows();
  Int numCols = matrix.cols();
  if (numRows != numCols) {
    throw InvalidArgumentException();
  }

  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueNoConditions(matrix(i, j), mat, node1Index + i,
                               node2Index + j, maxFreq, freqIdx, mSLog);
      }
    }
  } else if (isTerminal1NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueOnDiagonalNoConditions(matrix(i, j), mat, node1Index + i,
                                         maxFreq, freqIdx, mSLog);
      }
    }
  } else if (isTerminal2NotGrounded) {
    for (UInt i = 0; i < numRows; i++) {
      for (UInt j = 0; j < numCols; j++) {
        stampValueOnDiagonalNoConditions(matrix(i, j), mat, node2Index + j,
                                         maxFreq, freqIdx, mSLog);
      }
    }
  }
}

template <typename T>
void MNAStampUtils::stampValueNoConditions(T value, SparseMatrixRow &mat,
                                           UInt node1Index, UInt node2Index,
                                           Int maxFreq, Int freqIdx,
                                           const Logger::Log &mSLog) {
  stampValueOnDiagonalNoConditions(value, mat, node1Index, maxFreq, freqIdx,
                                   mSLog);
  stampValueOnDiagonalNoConditions(value, mat, node2Index, maxFreq, freqIdx,
                                   mSLog);
  stampValueOffDiagonalNoConditions(value, mat, node1Index, node2Index, maxFreq,
                                    freqIdx, mSLog);
}

template <typename T>
void MNAStampUtils::stampValueOnDiagonalNoConditions(T value,
                                                     SparseMatrixRow &mat,
                                                     UInt nodeIndex,
                                                     Int maxFreq, Int freqIdx,
                                                     const Logger::Log &mSLog) {
  addToMatrixElement(mat, nodeIndex, nodeIndex, value, maxFreq, freqIdx, mSLog);
}

template <typename T>
void MNAStampUtils::stampValueOffDiagonalNoConditions(
    T value, SparseMatrixRow &mat, UInt node1Index, UInt node2Index,
    Int maxFreq, Int freqIdx, const Logger::Log &mSLog) {
  addToMatrixElement(mat, node1Index, node2Index, -value, maxFreq, freqIdx,
                     mSLog);
  addToMatrixElement(mat, node2Index, node1Index, -value, maxFreq, freqIdx,
                     mSLog);
}

// These wrapper functions standardize the signatures of "Math::addToMatrixElement" for Real and Complex "value" parameters,
// facilitating the use of templates in the stamping logic.
void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Real value,
                                       Int maxFreq, Int freqIdx,
                                       const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog, "- Adding {:s} to system matrix element ({:d},{:d})",
                      Logger::realToString(value), row, column);

  Math::addToMatrixElement(mat, row, column, value);
}

void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Complex value,
                                       Int maxFreq, Int freqIdx,
                                       const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog, "- Adding {:s} to system matrix element ({:d},{:d})",
                      Logger::complexToString(value), row, column);

  Math::addToMatrixElement(mat, row, column, value, maxFreq, freqIdx);
}
