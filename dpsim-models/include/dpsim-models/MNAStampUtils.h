#pragma once

#include <dpsim-models/Logger.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {
class MNAStampUtils {
public:
  static void stampConductance(Real conductance, SparseMatrixRow &mat,
                               UInt node1Index, UInt node2Index,
                               Bool isTerminal1NotGrounded,
                               Bool isTerminal2NotGrounded,
                               const Logger::Log &mSLog);

  static void stampAdmittance(Complex admittance, SparseMatrixRow &mat,
                              UInt node1Index, UInt node2Index,
                              Bool isTerminal1NotGrounded,
                              Bool isTerminal2NotGrounded,
                              const Logger::Log &mSLog, Int maxFreq = 1,
                              Int freqIdx = 0);

  static void stampConductanceMatrix(const Matrix &conductanceMat,
                                     SparseMatrixRow &mat, UInt node1Index,
                                     UInt node2Index,
                                     Bool isTerminal1NotGrounded,
                                     Bool isTerminal2NotGrounded,
                                     const Logger::Log &mSLog);

  static void stamp3x3ConductanceMatrixBetween2Nodes(
      const Matrix &conductanceMat, SparseMatrixRow &mat, UInt node1Index,
      UInt node2Index, const Logger::Log &mSLog);

  static void stamp3x3ConductanceMatrixNodeToGround(const Matrix &conductanceMat,
                                             SparseMatrixRow &mat,
                                             UInt nodeIndex,
                                             const Logger::Log &mSLog);

  static void stampAdmittanceMatrix(
      const MatrixComp &admittanceMat, SparseMatrixRow &mat, UInt node1Index,
      UInt node2Index, Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
      const Logger::Log &mSLog, Int maxFreq = 1, Int freqIdx = 0);

  /// Stamps conductance as a 3x3 scalar matrix (a diagonal matrix, where all diagonal elements are equal to conductance).
  static void stampConductanceAs3x3ScalarMatrix(
      Real conductance, SparseMatrixRow &mat, UInt node1Index, UInt node2Index,
      Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
      const Logger::Log &mSLog);

  /// Stamps admittance as a 3x3 scalar matrix (a diagonal matrix, where all diagonal elements are equal to admittance).
  static void stampAdmittanceAs3x3ScalarMatrix(
      Complex admittance, SparseMatrixRow &mat, UInt node1Index,
      UInt node2Index, Bool isTerminal1NotGrounded, Bool isTerminal2NotGrounded,
      const Logger::Log &mSLog, Int maxFreq = 1, Int freqIdx = 0);

private:
  template <typename T>
  static void stampValue(T value, SparseMatrixRow &mat, UInt node1Index,
                         UInt node2Index, Bool isTerminal1NotGrounded,
                         Bool isTerminal2NotGrounded, Int maxFreq, Int freqIdx,
                         const Logger::Log &mSLog);

  template <typename T>
  static void stampMatrix(const MatrixVar<T> &matrix, SparseMatrixRow &mat,
                          UInt node1Index, UInt node2Index,
                          Bool isTerminal1NotGrounded,
                          Bool isTerminal2NotGrounded, Int maxFreq, Int freqIdx,
                          const Logger::Log &mSLog);

  template <typename T>
  static void stampMatrixBetween2Nodes(const MatrixVar<T> &matrix,
                                       UInt sizeOfMatrix, SparseMatrixRow &mat,
                                       UInt node1Index, UInt node2Index,
                                       Int maxFreq, Int freqIdx,
                                       const Logger::Log &mSLog);

  template <typename T>
  static void stampMatrixNodeToGround(const MatrixVar<T> &matrix,
                                      UInt sizeOfMatrix, SparseMatrixRow &mat,
                                      UInt nodeIndex, Int maxFreq, Int freqIdx,
                                      const Logger::Log &mSLog);

  template <typename T>
  static void stampValueAsScalarMatrix(T value, UInt sizeOfScalarMatrix,
                                       SparseMatrixRow &mat, UInt node1Index,
                                       UInt node2Index,
                                       Bool isTerminal1NotGrounded,
                                       Bool isTerminal2NotGrounded, Int maxFreq,
                                       Int freqIdx, const Logger::Log &mSLog);

  template <typename T>
  static void stampToMatrix(T value, SparseMatrixRow &mat, UInt row1,
                            UInt column1, UInt row2, UInt column2, Int maxFreq,
                            Int freqIdx, const Logger::Log &mSLog);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Real value, Int maxFreq,
                                 Int freqIdx, const Logger::Log &mSLog);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Complex value,
                                 Int maxFreq, Int freqIdx,
                                 const Logger::Log &mSLog);
};
} // namespace CPS
