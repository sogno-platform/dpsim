/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim-models/MathUtils.h>

using namespace CPS;

// #### Angular Operations ####
Real Math::radtoDeg(Real rad) { return rad * 180 / PI; }

Real Math::degToRad(Real deg) { return deg * PI / 180; }

Real Math::phase(Complex value) { return std::arg(value); }

Real Math::phaseDeg(Complex value) { return radtoDeg(phase(value)); }

Real Math::abs(Complex value) { return std::abs(value); }

Matrix Math::abs(const MatrixComp &mat) {
  size_t nRows = mat.rows();
  size_t nCols = mat.cols();
  Matrix res(mat.rows(), mat.cols());

  for (size_t i = 0; i < nRows; ++i) {
    for (size_t j = 0; j < nCols; ++j) {
      res(i, j) = std::abs(mat(i, j));
    }
  }
  return res;
}

Matrix Math::phase(const MatrixComp &mat) {
  size_t nRows = mat.rows();
  size_t nCols = mat.cols();
  Matrix res(mat.rows(), mat.cols());

  for (size_t i = 0; i < nRows; ++i) {
    for (size_t j = 0; j < nCols; ++j) {
      res(i, j) = std::arg(mat(i, j));
    }
  }
  return res;
}

Complex Math::polar(Real abs, Real phase) {
  return std::polar<Real>(abs, phase);
}

Complex Math::polarDeg(Real abs, Real phase) {
  return std::polar<Real>(abs, radtoDeg(phase));
}

void Math::setVectorElement(Matrix &mat, Matrix::Index row, Complex value,
                            Int maxFreq, Int freqIdx, Matrix::Index colOffset) {
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;

  mat(harmRow, colOffset) = value.real();
  mat(harmRow + complexOffset, colOffset) = value.imag();
}

void Math::addToVectorElement(Matrix &mat, Matrix::Index row, Complex value,
                              Int maxFreq, Int freqIdx) {
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;

  mat(harmRow, 0) = mat(harmRow, 0) + value.real();
  mat(harmRow + complexOffset, 0) =
      mat(harmRow + complexOffset, 0) + value.imag();
}

Complex Math::complexFromVectorElement(const Matrix &mat, Matrix::Index row,
                                       Int maxFreq, Int freqIdx) {
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;

  return Complex(mat(harmRow, 0), mat(harmRow + complexOffset, 0));
}

void Math::addToVectorElement(Matrix &mat, Matrix::Index row, Real value) {
  mat(row, 0) = mat(row, 0) + value;
}

void Math::setVectorElement(Matrix &mat, Matrix::Index row, Real value) {
  mat(row, 0) = value;
}

Real Math::realFromVectorElement(const Matrix &mat, Matrix::Index row) {
  return mat(row, 0);
}

void Math::setMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                            Matrix::Index column, Complex value, Int maxFreq,
                            Int freqIdx) {
  // Assume square matrix
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;
  Eigen::Index harmCol = column + harmonicOffset * freqIdx;

  mat.coeffRef(harmRow, harmCol) = value.real();
  mat.coeffRef(harmRow + complexOffset, harmCol + complexOffset) = value.real();
  mat.coeffRef(harmRow, harmCol + complexOffset) = -value.imag();
  mat.coeffRef(harmRow + complexOffset, harmCol) = value.imag();
}

void Math::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                              Matrix::Index column, Complex value, Int maxFreq,
                              Int freqIdx) {
  // Assume square matrix
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;
  Eigen::Index harmCol = column + harmonicOffset * freqIdx;

  mat.coeffRef(harmRow, harmCol) += value.real();
  mat.coeffRef(harmRow + complexOffset, harmCol + complexOffset) +=
      value.real();
  mat.coeffRef(harmRow, harmCol + complexOffset) -= value.imag();
  mat.coeffRef(harmRow + complexOffset, harmCol) += value.imag();
}

void Math::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                              Matrix::Index column, Matrix value, Int maxFreq,
                              Int freqIdx) {
  // Assume square matrix
  Eigen::Index harmonicOffset = mat.rows() / maxFreq;
  Eigen::Index complexOffset = harmonicOffset / 2;
  Eigen::Index harmRow = row + harmonicOffset * freqIdx;
  Eigen::Index harmCol = column + harmonicOffset * freqIdx;

  mat.coeffRef(harmRow, harmCol) += value(0, 0);
  mat.coeffRef(harmRow + complexOffset, harmCol + complexOffset) += value(1, 1);
  mat.coeffRef(harmRow, harmCol + complexOffset) += value(0, 1);
  mat.coeffRef(harmRow + complexOffset, harmCol) += value(1, 0);
}

void Math::setMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                            Matrix::Index column, Real value) {
  mat.coeffRef(row, column) = value;
}

void Math::addToMatrixElement(SparseMatrixRow &mat, std::vector<UInt> rows,
                              std::vector<UInt> columns, Complex value) {
  for (UInt phase = 0; phase < rows.size(); phase++)
    addToMatrixElement(mat, rows[phase], columns[phase], value);
}

void Math::addToMatrixElement(SparseMatrixRow &mat, Matrix::Index row,
                              Matrix::Index column, Real value) {
  mat.coeffRef(row, column) = mat.coeff(row, column) + value;
}

void Math::addToMatrixElement(SparseMatrixRow &mat, std::vector<UInt> rows,
                              std::vector<UInt> columns, Real value) {
  for (UInt phase = 0; phase < rows.size(); phase++)
    addToMatrixElement(mat, rows[phase], columns[phase], value);
}

void Math::invertMatrix(const Matrix &mat, Matrix &matInv) {
  const Int n = Eigen::internal::convert_index<Int>(mat.cols());
  if (n == 2) {
    const Real determinant = mat(0, 0) * mat(1, 1) - mat(0, 1) * mat(1, 0);
    matInv(0, 0) = mat(1, 1) / determinant;
    matInv(0, 1) = -mat(0, 1) / determinant;
    matInv(1, 0) = -mat(1, 0) / determinant;
    matInv(1, 1) = mat(0, 0) / determinant;
  } else if (n == 3) {
    const Real determinant =
        (mat(0, 0) * mat(1, 1) * mat(2, 2) + mat(0, 1) * mat(1, 2) * mat(2, 0) +
         mat(1, 0) * mat(2, 1) * mat(0, 2)) -
        (mat(2, 0) * mat(1, 1) * mat(0, 2) + mat(1, 0) * mat(0, 1) * mat(2, 2) +
         mat(2, 1) * mat(1, 2) * mat(0, 0));
    matInv(0, 0) =
        (mat(1, 1) * mat(2, 2) - mat(1, 2) * mat(2, 1)) / determinant;
    matInv(0, 1) =
        (mat(0, 2) * mat(2, 1) - mat(0, 1) * mat(2, 2)) / determinant;
    matInv(0, 2) =
        (mat(0, 1) * mat(1, 2) - mat(0, 2) * mat(1, 1)) / determinant;
    matInv(1, 0) =
        (mat(1, 2) * mat(2, 0) - mat(1, 0) * mat(2, 2)) / determinant;
    matInv(1, 1) =
        (mat(0, 0) * mat(2, 2) - mat(0, 2) * mat(2, 0)) / determinant;
    matInv(1, 2) =
        (mat(0, 2) * mat(1, 0) - mat(0, 0) * mat(1, 2)) / determinant;
    matInv(2, 0) =
        (mat(1, 0) * mat(2, 1) - mat(1, 1) * mat(2, 0)) / determinant;
    matInv(2, 1) =
        (mat(0, 1) * mat(2, 0) - mat(0, 0) * mat(2, 1)) / determinant;
    matInv(2, 2) =
        (mat(0, 0) * mat(1, 1) - mat(0, 1) * mat(1, 0)) / determinant;
  } else {
    matInv = mat.inverse();
  }
}

MatrixComp Math::singlePhaseVariableToThreePhase(Complex var_1ph) {
  MatrixComp var_3ph = MatrixComp::Zero(3, 1);
  var_3ph << var_1ph, var_1ph * SHIFT_TO_PHASE_B, var_1ph * SHIFT_TO_PHASE_C;
  return var_3ph;
}

Matrix Math::singlePhaseParameterToThreePhase(Real parameter) {
  Matrix param_3ph = Matrix::Zero(3, 3);
  param_3ph << parameter, 0., 0., 0., parameter, 0., 0, 0., parameter;
  return param_3ph;
}

Matrix Math::singlePhasePowerToThreePhase(Real power) {
  Matrix power_3ph = Matrix::Zero(3, 3);
  power_3ph << power / 3., 0., 0., 0., power / 3., 0., 0, 0., power / 3.;
  return power_3ph;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt,
                                   Matrix u_new, Matrix u_old) {
  Matrix::Index n = states.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  return F2inv * F1 * states + F2inv * (dt / 2.) * B * (u_new + u_old);
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C,
                                   Real dt, Matrix u_new, Matrix u_old) {
  Matrix::Index n = states.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  return F2inv * F1 * states + F2inv * (dt / 2.) * B * (u_new + u_old) +
         F2inv * dt * C;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C,
                                   Real dt, Matrix u) {
  Matrix::Index n = states.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  return F2inv * F1 * states + F2inv * dt * B * u + F2inv * dt * C;
}

Real Math::StateSpaceTrapezoidal(Real states, Real A, Real B, Real C, Real dt,
                                 Real u) {
  Real F1 = 1. + (dt / 2.) * A;
  Real F2 = 1. - (dt / 2.) * A;
  Real F2inv = 1. / F2;

  return F2inv * F1 * states + F2inv * dt * B * u + F2inv * dt * C;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt,
                                   Matrix u) {
  Matrix::Index n = states.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  return F2inv * F1 * states + F2inv * dt * B * u;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix input,
                                   Real dt) {
  Matrix::Index n = states.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  return F2inv * F1 * states + F2inv * dt * input;
}

Real Math::StateSpaceTrapezoidal(Real states, Real A, Real B, Real dt, Real u) {
  Real F1 = 1. + (dt / 2.) * A;
  Real F2 = 1. - (dt / 2.) * A;
  Real F2inv = 1. / F2;

  return F2inv * F1 * states + F2inv * dt * B * u;
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix B, Real dt,
                             Matrix u) {
  return states + dt * (A * states + B * u);
}

Real Math::StateSpaceEuler(Real states, Real A, Real B, Real dt, Real u) {
  return states + dt * (A * states + B * u);
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix B, Matrix C,
                             Real dt, Matrix u) {
  return states + dt * (A * states + B * u + C);
}

Real Math::StateSpaceEuler(Real states, Real A, Real B, Real C, Real dt,
                           Real u) {
  return states + dt * (A * states + B * u + C);
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix input, Real dt) {
  return states + dt * (A * states + input);
}

void Math::calculateStateSpaceTrapezoidalMatrices(const Matrix &A,
                                                  const Matrix &B,
                                                  const Matrix &C,
                                                  const Real &dt, Matrix &Ad,
                                                  Matrix &Bd, Matrix &Cd) {
  Matrix::Index n = A.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  Ad = F2inv * F1;
  Bd = F2inv * (dt / 2.) * B;
  Cd = F2inv * dt * C;
}

void Math::calculateStateSpaceTrapezoidalMatrices(const Matrix &A,
                                                  const Matrix &B,
                                                  const Real &dt, Matrix &Ad,
                                                  Matrix &Bd) {
  Matrix::Index n = A.rows();
  Matrix I = Matrix::Identity(n, n);

  Matrix F1 = I + (dt / 2.) * A;
  Matrix F2 = I - (dt / 2.) * A;
  Matrix F2inv = F2.inverse();

  Ad = F2inv * F1;
  Bd = F2inv * (dt / 2.) * B;
}

Matrix Math::applyStateSpaceTrapezoidalMatrices(const Matrix &Ad,
                                                const Matrix &Bd,
                                                const Matrix &Cd,
                                                const Matrix &statesPrevStep,
                                                const Matrix &inputCurrStep,
                                                const Matrix &inputPrevStep) {
  return Ad * statesPrevStep + Bd * (inputCurrStep + inputPrevStep) + Cd;
}

void Math::FFT(std::vector<Complex> &samples) {
  // DFT
  size_t N = samples.size();
  size_t k = N;
  size_t n;
  double thetaT = M_PI / N;
  Complex phiT = Complex(cos(thetaT), -sin(thetaT)), T;
  while (k > 1) {
    n = k;
    k >>= 1;
    phiT = phiT * phiT;
    T = 1.0L;
    for (size_t l = 0; l < k; l++) {
      for (size_t a = l; a < N; a += n) {
        size_t b = a + k;
        Complex t = samples[a] - samples[b];
        samples[a] += samples[b];
        samples[b] = t * T;
      }
      T *= phiT;
    }
  }
  // Decimate
  UInt m = static_cast<UInt>(log2(N));
  for (UInt a = 0; a < N; a++) {
    UInt b = a;
    // Reverse bits
    b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
    b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
    b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
    b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
    b = ((b >> 16) | (b << 16)) >> (32 - m);
    if (b > a) {
      Complex t = samples[a];
      samples[a] = samples[b];
      samples[b] = t;
    }
  }
}

Complex Math::rotatingFrame2to1(Complex f2, Real theta1, Real theta2) {
  Real delta = theta2 - theta1;
  Real f1_real = f2.real() * cos(delta) - f2.imag() * sin(delta);
  Real f1_imag = f2.real() * sin(delta) + f2.imag() * cos(delta);
  return Complex(f1_real, f1_imag);
}
