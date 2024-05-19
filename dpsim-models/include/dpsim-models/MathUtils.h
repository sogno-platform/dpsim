/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {

class Math {
public:
  typedef Real (*DeriveFnPtr)(Matrix inputs);

  // #### Angular Operations ####
  static Real radtoDeg(Real rad);

  static Real degToRad(Real deg);

  static Real phase(Complex value);

  static Real phaseDeg(Complex value);

  static Real abs(Complex value);

  static Matrix abs(const MatrixComp &mat);

  static Matrix phase(const MatrixComp &mat);

  static Complex polar(Real abs, Real phase);
  static Complex polarDeg(Real abs, Real phase);

  // #### Vector Operations ####
  //
  // | Re(row,0)_harm1 | Re(row,colOffset)_harm1 |
  // | Im(row,0)_harm1 | Im(row,colOffset)_harm1 |
  // | Re(row,0)_harm2 | Re(row,colOffset)_harm2 |
  // | Im(row,0)_harm2 | Im(row,colOffset)_harm2 |

  static void setVectorElement(Matrix &mat, Matrix::Index row, Complex value,
                               Int maxFreq = 1, Int freqIdx = 0,
                               Matrix::Index colOffset = 0);

  static void addToVectorElement(Matrix &mat, Matrix::Index row, Complex value,
                                 Int maxFreq = 1, Int freqIdx = 0);

  static Complex complexFromVectorElement(const Matrix &mat, Matrix::Index row,
                                          Int maxFreq = 1, Int freqIdx = 0);

  static void addToVectorElement(Matrix &mat, Matrix::Index row, Real value);
  static void setVectorElement(Matrix &mat, Matrix::Index row, Real value);

  static Real realFromVectorElement(const Matrix &mat, Matrix::Index row);

  // #### Matrix Operations ####
  //
  // | Re-Re(row,col)_harm1 | Im-Re(row,col)_harm1 | Interharmonics harm1-harm2
  // | Re-Im(row,col)_harm1 | Im-Im(row,col)_harm1 | Interharmonics harm1-harm2
  // | Interharmonics harm1-harm2                  | Re(row,col)_harm2 | Re(row,col)_harm2 |
  // | Interharmonics harm1-harm2                  | Im(row,col)_harm2 | Im(row,col)_harm2 |

  static MatrixComp returnNonZeroElements(const MatrixComp &mat);

  static MatrixComp convertRealEquivalentToComplexMatrix(const Matrix &realEquivalentMatrix);

  static void setMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                               Matrix::Index column, Complex value,
                               Int maxFreq = 1, Int freqIdx = 0);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Complex value,
                                 Int maxFreq = 1, Int freqIdx = 0);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Matrix value,
                                 Int maxFreq = 1, Int freqIdx = 0);

  static void setMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                               Matrix::Index column, Real value);

  static void addToMatrixElement(SparseMatrixRow &mat, std::vector<UInt> rows,
                                 std::vector<UInt> columns, Complex value);

  static void addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                                 Matrix::Index column, Real value);
  static void addToMatrixElement(SparseMatrixRow &mat, std::vector<UInt> rows,
                                 std::vector<UInt> columns, Real value);

  static void invertMatrix(const Matrix &mat, Matrix &matInv);

  // #### Integration Methods ####
  static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B,
                                      Real dt, Matrix u_new, Matrix u_old);
  static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B,
                                      Matrix C, Real dt, Matrix u_new,
                                      Matrix u_old);
  static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B,
                                      Real dt, Matrix u);
  static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B,
                                      Matrix C, Real dt, Matrix u);
  static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix input,
                                      Real dt);
  static Real StateSpaceTrapezoidal(Real states, Real A, Real B, Real C,
                                    Real dt, Real u);
  static Real StateSpaceTrapezoidal(Real states, Real A, Real B, Real dt,
                                    Real u);

  static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix B, Real dt,
                                Matrix u);
  static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix B, Matrix C,
                                Real dt, Matrix u);
  static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix input, Real dt);
  static Real StateSpaceEuler(Real states, Real A, Real B, Real dt, Real u);
  static Real StateSpaceEuler(Real states, Real A, Real B, Real C, Real dt,
                              Real u);

  /// Calculate the discretized state space matrices Ad, Bd, Cd using trapezoidal rule
  static void calculateStateSpaceTrapezoidalMatrices(const Matrix &A,
                                                     const Matrix &B,
                                                     const Matrix &C,
                                                     const Real &dt, Matrix &Ad,
                                                     Matrix &Bd, Matrix &Cd);
  /// Apply the trapezoidal based state space matrices Ad, Bd, Cd to get the states at the current time step
  static Matrix applyStateSpaceTrapezoidalMatrices(const Matrix &Ad,
                                                   const Matrix &Bd,
                                                   const Matrix &Cd,
                                                   const Matrix &statesPrevStep,
                                                   const Matrix &inputCurrStep,
                                                   const Matrix &inputPrevStep);

  static void FFT(std::vector<Complex> &samples);

  static Complex rotatingFrame2to1(Complex f2, Real theta1, Real theta2);

  /// To convert single phase complex variables (voltages, currents) to symmetrical three phase ones
  static MatrixComp singlePhaseVariableToThreePhase(Complex var_1ph);

  /// To convert single phase parameters to symmetrical three phase ones
  static Matrix singlePhaseParameterToThreePhase(Real parameter);

  /// To convert single phase power to symmetrical three phase
  static Matrix singlePhasePowerToThreePhase(Real power);
};
} // namespace CPS
