/* 
 * File:   Solution.h
 * Author: Santiago Peñate Vera
 *
 * Created on 8 de agosto de 2014, 10:43
 * Copyright (C) 2014 Santiago Peñate Vera
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include <iostream>
#include <ctime>

//EIGEN
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/LU>


//General math
#include <complex>
#include <cmath>

//Aseritions library
#include <assert.h> 


typedef std::complex<double> cx_double;

typedef Eigen::MatrixXd mat;
typedef Eigen::VectorXd vec;

typedef Eigen::VectorXcd cx_vec;
typedef Eigen::MatrixXcd cx_mat;

typedef Eigen::Matrix3cd cx_mat3; // 3x3 complex matrix

typedef Eigen::SparseMatrix<std::complex<double>,Eigen::RowMajor> sp_cx_mat;
typedef Eigen::SparseMatrix<double> sp_mat;
typedef Eigen::SparseVector<double>  sp_vec;

typedef Eigen::DiagonalMatrix<std::complex<double>,Eigen::Dynamic> cx_diag_mat;


typedef unsigned int uint;



using namespace std;

namespace DPsim {
#ifndef SOLUTION_H
#define	SOLUTION_H

    /*
     * This solution object with complex vectors is suited for methods like Jacobi
     * or Gauss-Seidel
     */
    class cx_solution {
    public:
        /*Properties*/
        cx_vec S;

        cx_vec V;

        bool initialized;

        uint Lenght;

        cx_solution();
        virtual ~cx_solution();

        void copy_from(cx_solution orig);

        void resize(int n);

        void print(string title);

        cx_vec getS();

        cx_vec getV();
        
        vec P();
        
        vec Q();
        
        double Pi(uint k);
        
        double Qi(uint k);

        double Vi(uint k);

        double Vr(uint k);

        mat getP();

    private:

    };

    /*
     * This type of separated vectors solution is good for solvers of the
     * Newton-Raphson type.
     */
    class solution {
    public:
        /*Properties*/
        vec P;

        vec Q;

        vec V;

        vec D;

        bool initialized;

        uint Lenght;

        solution();
        virtual ~solution();

        void copy_from(solution orig);

        void resize(int n);
        
        void clear();

        void print(string title);

        double Vi(uint k);

        double Vr(uint k);

        cx_double Vcx(uint k);

        cx_double Scx(uint k);

        cx_solution get_cx();
    private:

    };




#endif	/* SOLUTION_H */

}