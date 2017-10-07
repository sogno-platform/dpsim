/** Mathematics library
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
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

#ifndef MATHLIBRARY_H
#define MATHLIBRARY_H

#include <cmath>
#include <complex>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

// ### deprecated math section ###
typedef Eigen::MatrixXd DPSMatrix;

// VS2017 doesn't define M_PI unless _USE_MATH_DEFINES is included before cmath
// which is hard to guarantee, so we make sure it is defined here
#define DPS_PI PI
#ifndef PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PI M_PI
#endif

namespace DPsim
{
	// ### Constants ###

	// ### Types ###
	typedef unsigned int UInt;
	typedef int Int;
	typedef double Real;
	typedef std::complex<Real> Complex;
	static const Complex jComp(0.0, 1.0);

	/**
	* @brief Dense vector for complex numbers.
	*/
	typedef Eigen::Matrix<Complex, Eigen::Dynamic, 1> VectorComp;

	/**
	* @brief Dense vector for real numbers.
	*/
	typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> Vector;

	/**
	* @brief Sparse matrix for real numbers.
	*/
	typedef Eigen::SparseMatrix<Real, Eigen::ColMajor> SparseMatrix;

	/**
	* @brief Sparse matrix for complex numbers.
	*/
	typedef Eigen::SparseMatrix<Complex, Eigen::ColMajor> SparseMatrixComp;

	/**
	* @brief Dense matrix for real numbers.
	*/
	typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> Matrix;

	/**
	* @brief Dense matrix for complex numbers.
	*/
	typedef Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixComp;
}

#endif // !MATHLIBRARY_H