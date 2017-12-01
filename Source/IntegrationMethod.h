/** Integration methods
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

#pragma once

#include "Definitions.h"

namespace DPsim {

	typedef Real(*DeriveFnPtr) (Matrix inputs);

	Matrix Trapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u_new, Matrix u_old);
	Matrix Trapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u_new, Matrix u_old);
	Matrix Trapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u);
	Matrix Trapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u);
	Real Trapezoidal(Real states, Real A, Real B, Real C, Real dt, Real u);
	Real Trapezoidal(Real states, Real A, Real B, Real dt, Real u);
	Matrix Euler(Matrix states, Matrix A, Matrix B, Real dt, Matrix u);
	Matrix Euler(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u);
	Real Euler(Real states, Real A, Real B, Real dt, Real u);

}

