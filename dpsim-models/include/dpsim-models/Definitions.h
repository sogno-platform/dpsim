/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cmath>
#include <complex>
#include <string>
#include <memory>
#include <exception>
#include <cerrno>
#include <system_error>

#ifdef __clang__
#elif defined(__GNUC__)
  #pragma GCC diagnostic push
  #if __GNUC__ >= 7
    #pragma GCC diagnostic ignored "-Wint-in-bool-context" // https://eigen.tuxfamily.org/bz/show_bug.cgi?id=1402
  #endif
#endif

#include <Eigen/Dense>
#include <Eigen/Sparse>

#ifdef __clang__
#elif defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

// VS2017 doesn't define M_PI unless _USE_MATH_DEFINES is included before cmath
// which is hard to guarantee, so we make sure it is defined here
#define DPS_PI PI
#ifndef PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PI M_PI
#endif

#define SHIFT_TO_PHASE_B Complex(cos(-2 * M_PI / 3), sin(-2 * M_PI / 3))
#define SHIFT_TO_PHASE_C Complex(cos(2 * M_PI / 3), sin(2 * M_PI / 3))
#define RMS_TO_PEAK sqrt(2.)
#define PEAK_TO_RMS sqrt(1/2.)
#define RMS3PH_TO_PEAK1PH sqrt(2./3.)
#define PEAK1PH_TO_RMS3PH sqrt(3./2.)

#define P_SNUB_TRANSFORMER 1e-3		// 0.1% of rated power
#define Q_SNUB_TRANSFORMER 5e-4		// 0.05% of rated power

namespace CPS {

	// ### Types ###
	typedef unsigned int UInt;
	typedef int Int;
	typedef double Real;
	typedef std::complex<Real> Complex;
	typedef bool Bool;
	typedef std::string String;

	/// @brief Dense vector for complex numbers.
	typedef Eigen::Matrix<Complex, Eigen::Dynamic, 1> VectorComp;
	/// @brief Dense vector for real numbers.
	typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> Vector;
	/// @brief Sparse matrix for real numbers.
	typedef Eigen::SparseMatrix<Real, Eigen::ColMajor> SparseMatrix;
	/// @brief Sparse matrix for real numbers (row major).
	typedef Eigen::SparseMatrix<Real, Eigen::RowMajor> SparseMatrixRow;
	/// @brief Sparse matrix for complex numbers.
	typedef Eigen::SparseMatrix<Complex, Eigen::ColMajor> SparseMatrixComp;
	/// @brief Sparse matrix for complex numbers (row major).
	typedef Eigen::SparseMatrix<Complex, Eigen::RowMajor> SparseMatrixCompRow;
	/// @brief Dense matrix for real numbers.
	typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> Matrix;
	/// @brief Dense matrix for complex numbers.
	typedef Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixComp;
	/// @brief Dense matrix for integers.
	typedef Eigen::Matrix<Int, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixInt;
	///
	typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRow;
	///
	typedef Eigen::PartialPivLU<Matrix> LUFactorized;
	///
	typedef Eigen::SparseLU<SparseMatrix> LUFactorizedSparse;
	///
	typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> Vector;
	///
	template<typename VarType>
	using MatrixVar = Eigen::Matrix<VarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
	/// @brief Dense matrix for real numbers with fixed dimension.
	template<int rows, int cols>
	using MatrixFixedSize = Eigen::Matrix<Real, rows, cols, Eigen::ColMajor>;
	/// @brief Dense matrix for complex numbers with fixed dimension.
	template<int rows, int cols>
	using MatrixFixedSizeComp = Eigen::Matrix<Complex, rows, cols, Eigen::ColMajor>;

	// ### Constants ###
	/// @cond WORKAROUND (Otherwise this code breaks Breathe / Doxygen)
	static const Complex jComp(0.0, 1.0);
	/// @endcond

	// ### Enumerations ###
	enum class NumericalMethod { Euler, Trapezoidal };
	enum class PhaseType { A, B, C, ABC, Single };
	enum class Domain { SP, DP, EMT };
	enum class PowerflowBusType { PV, PQ, VD, None };
	enum class GeneratorType {PVNode, IdealVoltageSource, IdealCurrentSource, TransientStability, FullOrder, FullOrderVBR, SG6aOrderVBR, SG6bOrderVBR, SG5OrderVBR, SG4OrderVBR, SG3OrderVBR, SG4OrderPCM, SG4OrderTPM, SG6OrderPCM, None};
	enum class SGOrder {SG3Order, SG4Order, SG5Order, SG6aOrder, SG6bOrder};
	enum class ExciterType {Simple, DC1Simp, DC1, ST1Simp};
	
	// ### Exceptions ###
	class Exception : public std::exception { };
	class AccessException : public Exception { };
	class TypeException : public Exception { };
	class InvalidAttributeException : public Exception { };
	class InvalidArgumentException : public Exception { };

	class SystemError {
	protected:
		std::error_code mErrorCode;
		String mDescription;
	public:
		SystemError(const String& desc, int e) :
			mErrorCode(e, std::system_category()),
			mDescription(desc) { };

		SystemError(const String& desc) :
			mErrorCode(errno, std::system_category()),
			mDescription(desc) { };

		SystemError() :
			mErrorCode(errno, std::system_category()) { };

		String descr() const {
			std::ostringstream os;

			os << "System Error: " << mDescription << ": " << mErrorCode.message() << "(" << mErrorCode.value() << ")";

			return os.str();
		};
	};
}
