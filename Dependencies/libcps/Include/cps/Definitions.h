/** Definitions
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
	typedef Eigen::PartialPivLU<Matrix> LUFactorized;
	///
	typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> Vector;
	///
	template<typename VarType>
	using MatrixVar = Eigen::Matrix<VarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

	// ### Constants ###
	/// @cond WORKAROUND (Otherwise this code breaks Breathe / Doxygen)
	static const Complex jComp(0.0, 1.0);
	/// @endcond

	// ### Enumerations ###
	enum class NumericalMethod { Euler, Trapezoidal };
	enum class PhaseType { A, B, C, ABC, Single };
	enum class Domain { SP, DP, EMT };
	enum class PowerflowBusType { PV, PQ, VD, None };

	// ### Exceptions ###
	class Exception : public std::exception { };
	class AccessException : public Exception { };
	class TypeException : public Exception { };
	class InvalidAttributeException : public Exception { };

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
