#ifndef MATHLIBRARY_H
#define MATHLIBRARY_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

// ### deprecated math section ###
typedef Eigen::MatrixXd DPSMatrix;
#define DPS_PI M_PI

namespace DPsim
{
	// ### Constants ###
	#define DPsim_PI M_PI

	// ### Types ###
	typedef uint64_t UInt;
	typedef int64_t Int;
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