/* 
 * File:   Solution.cpp
 * Author: Santiago Peñate Vera
 * 
 * Created on 8 de agosto de 2014, 10:43
 * Copyright (C) 2014 Santiago Peñate Vera
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <vector>

#include "dpsim/NRPSolution.h"

namespace DPsim {
    /***************************************************************************
     *Polar voltage solution
     
     * This is the implementation of the solution object with voltage in polar 
     * form. This way of representation of the voltage is usefull for the Newton
     * -Raphson algorithm, in order to better represent the PV buses constraint
     **************************************************************************/

    /*
     * solution object constructor
     */
    solution::solution() {
        initialized = false;
    }

    /*
     * soluton object destructor
     */
    solution::~solution() {
    }

    /*
     * this function initializes this solution object as a copy of another
     * solution object provided
     */
    void solution::copy_from(solution orig) {
        P.swap(orig.P);
        Q.swap(orig.Q);
        V.swap(orig.V);
        D.swap(orig.D);
    }

    /*
     * This function, resizes the solution object and initilaizes it to zero
     */
    void solution::resize(int n) {
        P = vec(n);
        Q = vec(n);
        V = vec(n);
        D = vec(n);
        P.setZero(n);
        Q.setZero(n);
        V.setZero(n);
        D.setZero(n);
        Lenght = n;
    }

    /*
     * set zero to the container vectors
     */
    void solution::clear() {
        P.setZero(Lenght);
        Q.setZero(Lenght);
        V.setZero(Lenght);
        D.setZero(Lenght);
    }

    /*
     * Complex power
     */
    cx_double solution::Scx(uint k) {
        return cx_double(P.coeff(k), Q.coeff(k));
    }

    /*
     * Real voltage
     */
    double solution::Vr(uint k) {
        return V.coeff(k) * cos(D.coeff(k));
    }

    /*
     * Imaginary voltage
     */
    double solution::Vi(uint k) {
        return V.coeff(k) * sin(D.coeff(k));
    }

    /*
     * Complex voltage
     */
    cx_double solution::Vcx(uint k) {
        return cx_double(Vr(k), Vi(k));
    }

    /*
     * Complex solution
     */
    cx_solution solution::get_cx() {
        cx_solution sol;
        sol.resize(Lenght);
        for (uint i = 0; i < Lenght; i++) {
            sol.S(i) = Scx(i);
            sol.V(i) = Vcx(i);
        }
        return sol;
    }

    /*real power vector
     */
    vec cx_solution::P() {
        vec val(Lenght);
        for (uint i = 0; i < Lenght; i++)
            val(i) = S.coeff(i).real();
        return val;
    }

    /*Imaginary power vector
     */
    vec cx_solution::Q() {
        vec val(Lenght);
        for (uint i = 0; i < Lenght; i++)
            val(i) = S.coeff(i).imag();
        return val;
    }

    /*
     * This function prints the solution values
     */
    void solution::print(string title) {
        cout << title << endl;
        cout << "P\tQ\tV\tD" << endl;
        for (uint i = 0; i < Lenght; i++) {
            cout << P[i] << "\t" << Q[i] << "\t" << V[i] << "\t" << D[i] << endl;
        }
    }

    /***************************************************************************
     *Complex voltage and power solution
     * 
     * This implementation of the solution object contains both voltage and 
     * power in complex mode. This is the solution "shape" that suits best
     * algorithms like gauss-seidel
     **************************************************************************/

    /*
     * cxsolution object constructor
     */
    cx_solution::cx_solution() {
        initialized = false;
    }

    /*
     * cx_solution object destructor
     */
    cx_solution::~cx_solution() {
    }

    /*
     * this function initializes this cx_solution object as a copy of another
     * cx_solution object provided
     */
    void cx_solution::copy_from(cx_solution orig) {
        S.swap(orig.S);
        V.swap(orig.V);
    }

    /*
     * This function, resizes the solution object and initilaizes it to zero
     */
    void cx_solution::resize(int n) {
        S = cx_vec(n);
        V = cx_vec(n);
        S.setZero(n);
        V.setZero(n);
        Lenght = n;
    }

    /*
     * This function prints the solution values
     */
    void cx_solution::print(string title) {
        cout << title << endl;
        cout << "S\t\tV" << endl;
        for (uint i = 0; i < Lenght; i++) {
            cout << S[i] << "\t\t" << V[i] << "->" << abs(V[i]) << "^" << arg(V[i]) << endl;
        }
    }

    /*
     * TH function returns a n x 1 matrix containing the complex power
     */
    cx_vec cx_solution::getS() {
        cx_vec s(Lenght, 1);
        for (uint i = 0; i < Lenght; i++)
            s(i) = S.coeff(i);

        return s;
    }

    /*
     * This function returns a n x 1 matrix containing the onplex voltage
     */
    cx_vec cx_solution::getV() {
        cx_vec s(Lenght);
        for (uint i = 0; i < Lenght; i++)
            s(i) = V.coeff(i);

        return s;
    }

    /*
     * This function returs a vector containing the active power
     */
    mat cx_solution::getP() {
        mat s(Lenght, 1);
        for (uint i = 0; i < Lenght; i++)
            s(i, 0) = S.coeff(i).real();

        return s;
    }
    
    /*
     * This function returns the imaginary part of the voltage at the index k
     */
    double cx_solution::Pi(uint k) {
        return S.coeff(k).real();
    }
    
    
    /*
     * This function returns the imaginary part of the voltage at the index k
     */
    double cx_solution::Qi(uint k) {
        return S.coeff(k).imag();
    }

    /*
     * This function returns the imaginary part of the voltage at the index k
     */
    double cx_solution::Vi(uint k) {
        return V.coeff(k).imag();
    }

    /*
     * This function returns the real part of the voltage at the index k
     */
    double cx_solution::Vr(uint k) {
        return V.coeff(k).real();
    }

}
