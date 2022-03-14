/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Definitions.h>

namespace CPS {

	class Math {
	public:
		typedef Real(*DeriveFnPtr) (Matrix inputs);

		// #### Angular Operations ####
		static Real radtoDeg(Real rad) {
			return rad * 180 / PI;
		}

		static Real degToRad(Real deg) {
			return deg * PI / 180;
		}

		static Real phase(Complex value) {
			return std::arg(value);
		}

		static Real phaseDeg(Complex value) {
			return radtoDeg(phase(value));
		}

		static Real abs(Complex value) {
			return std::abs(value);
		}

		static Matrix abs(const MatrixComp& mat) {
			size_t nRows = mat.rows();
			size_t nCols = mat.cols();
			Matrix res(mat.rows(), mat.cols());

			for (size_t i = 0; i < nRows; ++i) {
				for (size_t j = 0; j < nCols; ++j) {
					res(i,j) = std::abs(mat(i,j));
				}
			}
			return res;
		}

		static Matrix phase(const MatrixComp& mat) {
			size_t nRows = mat.rows();
			size_t nCols = mat.cols();
			Matrix res(mat.rows(), mat.cols());

			for (size_t i = 0; i < nRows; ++i) {
				for (size_t j = 0; j < nCols; ++j) {
					res(i,j) = std::arg(mat(i,j));
				}
			}
			return res;
		}

		static Complex polar(Real abs, Real phase) {
			return std::polar<Real>(abs, phase);
		}

		static Complex polarDeg(Real abs, Real phase) {
			return std::polar<Real>(abs, radtoDeg(phase));
		}

		// #### Vector Operations ####
		//
		// | Re(row,0)_harm1 | Re(row,colOffset)_harm1 |
		// | Im(row,0)_harm1 | Im(row,colOffset)_harm1 |
		// | Re(row,0)_harm2 | Re(row,colOffset)_harm2 |
		// | Im(row,0)_harm2 | Im(row,colOffset)_harm2 |

		static void setVectorElement(Matrix& mat, Matrix::Index row, Complex value, Int maxFreq = 1, Int freqIdx = 0, Matrix::Index colOffset = 0) {
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;

			mat(harmRow, colOffset) = value.real();
			mat(harmRow + complexOffset, colOffset) = value.imag();
		}

		static void addToVectorElement(Matrix& mat, Matrix::Index row, Complex value, Int maxFreq = 1, Int freqIdx = 0) {
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;

			mat(harmRow, 0) = mat(harmRow, 0) + value.real();
			mat(harmRow + complexOffset, 0) = mat(harmRow + complexOffset, 0) + value.imag();
		}

		static Complex complexFromVectorElement(const Matrix& mat, Matrix::Index row, Int maxFreq = 1, Int freqIdx = 0) {
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;

			return Complex(mat(harmRow, 0), mat(harmRow + complexOffset, 0));
		}

		static void addToVectorElement(Matrix& mat, Matrix::Index row, Real value) {
			mat(row, 0) = mat(row, 0) + value;
		}

		static void setVectorElement(Matrix& mat, Matrix::Index row, Real value) {
			mat(row, 0) = value;
		}

		static Real realFromVectorElement(const Matrix& mat, Matrix::Index row) {
			return mat(row, 0);
		}

		// #### Matric Operations ####
		//
		// | Re-Re(row,col)_harm1 | Im-Re(row,col)_harm1 | Interharmonics harm1-harm2
		// | Re-Im(row,col)_harm1 | Im-Im(row,col)_harm1 | Interharmonics harm1-harm2
		// | Interharmonics harm1-harm2                  | Re(row,col)_harm2 | Re(row,col)_harm2 |
		// | Interharmonics harm1-harm2                  | Im(row,col)_harm2 | Im(row,col)_harm2 |

		static void setMatrixElement(Matrix& mat, Matrix::Index row, Matrix::Index column, Complex value, Int maxFreq = 1, Int freqIdx = 0) {
			// Assume square matrix
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;
			Eigen::Index harmCol = column + harmonicOffset * freqIdx;

			mat(harmRow, harmCol) = value.real();
			mat(harmRow + complexOffset, harmCol + complexOffset) = value.real();
			mat(harmRow, harmCol + complexOffset) = - value.imag();
			mat(harmRow + complexOffset, harmCol) = value.imag();
		}

		static void addToMatrixElement(Matrix& mat, Matrix::Index row, Matrix::Index column, Complex value, Int maxFreq = 1, Int freqIdx = 0) {
			// Assume square matrix
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;
			Eigen::Index harmCol = column + harmonicOffset * freqIdx;

			mat(harmRow, harmCol) = mat(harmRow, harmCol) + value.real();
			mat(harmRow + complexOffset, harmCol + complexOffset) = mat(harmRow + complexOffset, harmCol + complexOffset) + value.real();
			mat(harmRow, harmCol + complexOffset) = mat(harmRow, harmCol + complexOffset) - value.imag();
			mat(harmRow + complexOffset, harmCol) = mat(harmRow + complexOffset, harmCol) + value.imag();
		}

		static void addToMatrixElement(Matrix& mat, Matrix::Index row, Matrix::Index column, Matrix value, Int maxFreq = 1, Int freqIdx = 0) {
			// Assume square matrix
			Eigen::Index harmonicOffset = mat.rows() / maxFreq;
			Eigen::Index complexOffset = harmonicOffset / 2;
			Eigen::Index harmRow = row + harmonicOffset * freqIdx;
			Eigen::Index harmCol = column + harmonicOffset * freqIdx;

			mat(harmRow, harmCol) = mat(harmRow, harmCol) + value(0,0);
			mat(harmRow + complexOffset, harmCol + complexOffset) = mat(harmRow + complexOffset, harmCol + complexOffset) + value(1,1);
			mat(harmRow, harmCol + complexOffset) = mat(harmRow, harmCol + complexOffset) + value(0,1);
			mat(harmRow + complexOffset, harmCol) = mat(harmRow + complexOffset, harmCol) + value(1,0);
		}

		static void setMatrixElement(Matrix& mat, Matrix::Index row, Matrix::Index column, Real value) {
			mat(row, column) =  value;
		}

		static void addToMatrixElement(Matrix& mat, std::vector<UInt> rows, std::vector<UInt> columns, Complex value) {
			for (UInt phase = 0; phase < rows.size(); phase++)
					addToMatrixElement(mat, rows[phase], columns[phase], value);
		}

		static void addToMatrixElement(Matrix& mat, Matrix::Index row, Matrix::Index column, Real value) {
			mat(row, column) = mat(row, column) + value;
		}

		static void addToMatrixElement(Matrix& mat, std::vector<UInt> rows, std::vector<UInt> columns, Real value) {
			for (UInt phase = 0; phase < rows.size(); phase++)
					addToMatrixElement(mat, rows[phase], columns[phase], value);
		}

		// #### Integration Methods ####
		static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u_new, Matrix u_old);
		static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u_new, Matrix u_old);
		static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u);
		static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u);
		static Matrix StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix input, Real dt);
		static Real StateSpaceTrapezoidal(Real states, Real A, Real B, Real C, Real dt, Real u);
		static Real StateSpaceTrapezoidal(Real states, Real A, Real B, Real dt, Real u);

		static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix B, Real dt, Matrix u);
		static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u);
		static Matrix StateSpaceEuler(Matrix states, Matrix A, Matrix input, Real dt);
		static Real StateSpaceEuler(Real states, Real A, Real B, Real dt, Real u);
		static Real StateSpaceEuler(Real states, Real A, Real B, Real C, Real dt, Real u);

		static void FFT(std::vector<Complex>& samples);

		static Complex rotatingFrame2to1(Complex f2, Real theta1, Real theta2);

		/// To convert single phase complex variables (voltages, currents) to symmetrical three phase ones
		static MatrixComp singlePhaseVariableToThreePhase(Complex var_1ph) {
			MatrixComp var_3ph = MatrixComp::Zero(3, 1);
			var_3ph <<
				var_1ph,
				var_1ph * SHIFT_TO_PHASE_B,
				var_1ph * SHIFT_TO_PHASE_C;
			return var_3ph;
		}

		/// To convert single phase parameters to symmetrical three phase ones
		static Matrix singlePhaseParameterToThreePhase(Real parameter) {
			Matrix param_3ph = Matrix::Zero(3, 3);
			param_3ph <<
				parameter, 0., 0.,
				0., parameter, 0.,
				0, 0., parameter;
			return param_3ph;
		}

		/// To convert single phase power to symmetrical three phase
		static Matrix singlePhasePowerToThreePhase(Real power) {
			Matrix power_3ph = Matrix::Zero(3, 3);
			power_3ph <<
				power/3., 0., 0.,
				0., power/3., 0.,
				0, 0., power/3.;
			return power_3ph;
		}
	};
}
