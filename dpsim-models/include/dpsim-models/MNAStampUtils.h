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

  static void stampAdmittanceMatrix(
      const MatrixComp &admittanceMat, SparseMatrixRow &mat, UInt node1Index,
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
  static void stampValueNoConditions(T value, SparseMatrixRow &mat,
                                     UInt node1Index, UInt node2Index,
                                     Int maxFreq, Int freqIdx,
                                     const Logger::Log &mSLog);

  template <typename T>
  static void stampValueOnDiagonalNoConditions(T value, SparseMatrixRow &mat,
                                               UInt nodeIndex, Int maxFreq,
                                               Int freqIdx,
                                               const Logger::Log &mSLog);

  template <typename T>
  static void stampValueOffDiagonalNoConditions(T value, SparseMatrixRow &mat,
                                                UInt node1Index,
                                                UInt node2Index, Int maxFreq,
                                                Int freqIdx,
                                                const Logger::Log &mSLog);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Real value, Int maxFreq,
                                 Int freqIdx, const Logger::Log &mSLog);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Complex value,
                                 Int maxFreq, Int freqIdx,
                                 const Logger::Log &mSLog);
};
} // namespace CPS