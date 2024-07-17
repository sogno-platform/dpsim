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
      mSLog, "Start stamping admittance for frequency index {:d}...", freqIdx);

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

void MNAStampUtils::stamp3x3ConductanceMatrixBetween2Nodes(
    const Matrix &conductanceMat, SparseMatrixRow &mat, UInt node1Index,
    UInt node2Index, const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(
      mSLog, "Start stamping 3x3 conductance matrix between two nodes...");

  stampMatrixBetween2Nodes(conductanceMat, 3, mat, node1Index, node2Index, 1, 0,
                           mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stamp3x3ConductanceMatrixNodeToGround(
    const Matrix &conductanceMat, SparseMatrixRow &mat, UInt nodeIndex,
    const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(
      mSLog, "Start stamping 3x3 conductance matrix from node to ground...");

  stampMatrixNodeToGround(conductanceMat, 3, mat, nodeIndex, 1, 0, mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampAdmittanceMatrix(
    const MatrixComp &admittanceMat, SparseMatrixRow &mat, UInt node1Index,
    UInt node2Index, Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
    const Logger::Log &mSLog, Int maxFreq, Int freqIdx) {
  SPDLOG_LOGGER_DEBUG(
      mSLog, "Start stamping admittance matrix for frequency index {:d}...",
      freqIdx);

  stampMatrix(admittanceMat, mat, node1Index, node2Index,
              isTerminal1NotGrounded, isTerminal2NotGrounded, maxFreq, freqIdx,
              mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampConductanceAs3x3ScalarMatrix(
    Real conductance, SparseMatrixRow &mat, UInt node1Index, UInt node2Index,
    Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
    const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog,
                      "Start stamping conductance as 3x3 scalar matrix...");

  stampValueAsScalarMatrix(conductance, 3, mat, node1Index, node2Index,
                           isTerminal1NotGrounded, isTerminal2NotGrounded, 1, 0,
                           mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

void MNAStampUtils::stampAdmittanceAs3x3ScalarMatrix(
    Complex admittance, SparseMatrixRow &mat, UInt node1Index, UInt node2Index,
    Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
    const Logger::Log &mSLog, Int maxFreq, Int freqIdx) {
  SPDLOG_LOGGER_DEBUG(mSLog,
                      "Start stamping admittance as 3x3 scalar matrix for "
                      "frequency index {:d}...",
                      freqIdx);

  stampValueAsScalarMatrix(admittance, 3, mat, node1Index, node2Index,
                           isTerminal1NotGrounded, isTerminal2NotGrounded,
                           maxFreq, freqIdx, mSLog);

  SPDLOG_LOGGER_DEBUG(mSLog, "Stamping completed.");
}

template <typename T>
void MNAStampUtils::stampValue(T value, SparseMatrixRow &mat, UInt node1Index,
                               UInt node2Index, Bool isTerminal1NotGrounded,
                               Bool isTerminal2NotGrounded, Int maxFreq,
                               Int freqIdx, const Logger::Log &mSLog) {
  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    stampToMatrix(value, mat, node1Index, node1Index, node2Index, node2Index,
                  maxFreq, freqIdx, mSLog);
  } else if (isTerminal1NotGrounded) {
    addToMatrixElement(mat, node1Index, node1Index, value, maxFreq, freqIdx,
                       mSLog);
  } else if (isTerminal2NotGrounded) {
    addToMatrixElement(mat, node2Index, node2Index, value, maxFreq, freqIdx,
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
    stampMatrixBetween2Nodes(matrix, numRows, mat, node1Index, node2Index,
                             maxFreq, freqIdx, mSLog);
  } else if (isTerminal1NotGrounded) {
    stampMatrixNodeToGround(matrix, numRows, mat, node1Index, maxFreq, freqIdx,
                            mSLog);
  } else if (isTerminal2NotGrounded) {
    stampMatrixNodeToGround(matrix, numRows, mat, node2Index, maxFreq, freqIdx,
                            mSLog);
  }
}

template <typename T>
void MNAStampUtils::stampMatrixBetween2Nodes(const MatrixVar<T> &matrix,
                                             UInt sizeOfMatrix,
                                             SparseMatrixRow &mat,
                                             UInt node1Index, UInt node2Index,
                                             Int maxFreq, Int freqIdx,
                                             const Logger::Log &mSLog) {
  for (UInt i = 0; i < sizeOfMatrix; i++) {
    for (UInt j = 0; j < sizeOfMatrix; j++) {
      stampToMatrix(matrix(i, j), mat, node1Index + i, node1Index + j,
                    node2Index + i, node2Index + j, maxFreq, freqIdx, mSLog);
    }
  }
}

template <typename T>
void MNAStampUtils::stampMatrixNodeToGround(
    const MatrixVar<T> &matrix, UInt sizeOfMatrix, SparseMatrixRow &mat,
    UInt nodeIndex, Int maxFreq, Int freqIdx, const Logger::Log &mSLog) {
  for (UInt i = 0; i < sizeOfMatrix; i++) {
    for (UInt j = 0; j < sizeOfMatrix; j++) {
      addToMatrixElement(mat, nodeIndex + i, nodeIndex + j, matrix(i, j),
                         maxFreq, freqIdx, mSLog);
    }
  }
}

template <typename T>
void MNAStampUtils::stampValueAsScalarMatrix(
    T value, UInt sizeOfScalarMatrix, SparseMatrixRow &mat, UInt node1Index,
    UInt node2Index, Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
    Int maxFreq, Int freqIdx, const Logger::Log &mSLog) {
  if (isTerminal1NotGrounded && isTerminal2NotGrounded) {
    for (UInt i = 0; i < sizeOfScalarMatrix; i++) {
      stampToMatrix(value, mat, node1Index + i, node1Index + i, node2Index + i,
                    node2Index + i, maxFreq, freqIdx, mSLog);
    }
  } else if (isTerminal1NotGrounded) {
    for (UInt i = 0; i < sizeOfScalarMatrix; i++) {
      addToMatrixElement(mat, node1Index + i, node1Index + i, value, maxFreq,
                         freqIdx, mSLog);
    }
  } else if (isTerminal2NotGrounded) {
    for (UInt i = 0; i < sizeOfScalarMatrix; i++) {
      addToMatrixElement(mat, node2Index + i, node2Index + i, value, maxFreq,
                         freqIdx, mSLog);
    }
  }
}

template <typename T>
void MNAStampUtils::stampToMatrix(T value, SparseMatrixRow &mat, UInt row1,
                                  UInt column1, UInt row2, UInt column2,
                                  Int maxFreq, Int freqIdx,
                                  const Logger::Log &mSLog) {
  addToMatrixElement(mat, row1, column1, value, maxFreq, freqIdx, mSLog);
  addToMatrixElement(mat, row1, column2, -value, maxFreq, freqIdx, mSLog);
  addToMatrixElement(mat, row2, column1, -value, maxFreq, freqIdx, mSLog);
  addToMatrixElement(mat, row2, column2, value, maxFreq, freqIdx, mSLog);
}

// These wrapper functions standardize the signatures of "Math::addToMatrixElement" for Real and Complex "value" parameters,
// facilitating the use of templates in the stamping logic.
void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Real value,
                                       Int maxFreq, Int freqIdx,
                                       const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog,
                      "- Adding {:s} to system matrix element ({:d},{:d})",
                      Logger::realToString(value), row, column);

  Math::addToMatrixElement(mat, row, column, value);
}

void MNAStampUtils::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                       Matrix::Index column, Complex value,
                                       Int maxFreq, Int freqIdx,
                                       const Logger::Log &mSLog) {
  SPDLOG_LOGGER_DEBUG(mSLog,
                      "- Adding {:s} to system matrix element ({:d},{:d})",
                      Logger::complexToString(value), row, column);

  Math::addToMatrixElement(mat, row, column, value, maxFreq, freqIdx);
}
