/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cps/MathUtils.h>

using namespace CPS;

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u_new, Matrix u_old) {
	Matrix::Index n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix F1 = I + (dt/2.) * A;
	Matrix F2 = I - (dt/2.) * A;
	Matrix F2inv = F2.inverse();

	return F2inv*F1*states + F2inv*(dt/2.) * B*(u_new + u_old);
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u_new, Matrix u_old) {
	Matrix::Index n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix F1 = I + (dt/2.) * A;
	Matrix F2 = I - (dt/2.) * A;
	Matrix F2inv = F2.inverse();

	return F2inv*F1*states + F2inv*(dt/2.) * B*(u_new + u_old) + F2inv*dt*C;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u) {
	Matrix::Index n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix F1 = I + (dt/2.) * A;
	Matrix F2 = I - (dt/2.) * A;
	Matrix F2inv = F2.inverse();

	return F2inv*F1*states + F2inv*dt*B*u + F2inv*dt*C;
}

Real Math::StateSpaceTrapezoidal(Real states, Real A, Real B, Real C, Real dt, Real u) {
	Real F1 = 1. + (dt/2.) * A;
	Real F2 = 1. - (dt/2.) * A;
	Real F2inv = 1. / F2;

	return F2inv*F1*states + F2inv*dt*B*u + F2inv*dt*C;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u) {
	Matrix::Index n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix F1 = I + (dt/2.) * A;
	Matrix F2 = I - (dt/2.) * A;
	Matrix F2inv = F2.inverse();

	return F2inv * F1*states + F2inv * dt*B*u;
}

Matrix Math::StateSpaceTrapezoidal(Matrix states, Matrix A, Matrix input, Real dt) {
	Matrix::Index n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix F1 = I + (dt/2.) * A;
	Matrix F2 = I - (dt/2.) * A;
	Matrix F2inv = F2.inverse();

	return F2inv*F1*states + F2inv*dt*input;
}

Real Math::StateSpaceTrapezoidal(Real states, Real A, Real B, Real dt, Real u) {
	Real F1 = 1. + (dt/2.) * A;
	Real F2 = 1. - (dt/2.) * A;
	Real F2inv = 1. / F2;

	return F2inv * F1*states + F2inv * dt*B*u;
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix B, Real dt, Matrix u) {
	return states + dt * ( A*states + B*u );
}

Real Math::StateSpaceEuler(Real states, Real A, Real B, Real dt, Real u) {
	return states + dt * ( A*states + B*u );
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u) {
	return states + dt * ( A*states + B*u + C );
}

Real Math::StateSpaceEuler(Real states, Real A, Real B, Real C, Real dt, Real u) {
	return states + dt * ( A*states + B*u + C );
}

Matrix Math::StateSpaceEuler(Matrix states, Matrix A, Matrix input, Real dt) {
	return states + dt * ( A*states + input );
}

void Math::FFT(std::vector<Complex>& samples) {
	// DFT
	size_t N = samples.size();
	size_t k = N;
	size_t n;
	double thetaT = M_PI / N;
	Complex phiT = Complex(cos(thetaT), -sin(thetaT)), T;
	while (k > 1)
	{
		n = k;
		k >>= 1;
		phiT = phiT * phiT;
		T = 1.0L;
		for (size_t l = 0; l < k; l++)
		{
			for (size_t a = l; a < N; a += n)
			{
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
	for (UInt a = 0; a < N; a++)
	{
		UInt b = a;
		// Reverse bits
		b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
		b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
		b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
		b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
		b = ((b >> 16) | (b << 16)) >> (32 - m);
		if (b > a)
		{
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
