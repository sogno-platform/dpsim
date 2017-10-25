/** Integration methods
 *
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

#include "IntegrationMethod.h"

using namespace DPsim;

Matrix DPsim::Trapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u_new, Matrix u_old)
{
	int n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix Aux = I + (dt / 2) * A;
	Matrix Aux2 = I - (dt / 2) * A;

	Matrix newstates = Aux2.inverse()*Aux*states + Aux2.inverse()*(dt / 2) * B*(u_new + u_old);
	return newstates;
}

Matrix DPsim::Trapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u_new, Matrix u_old)
{
	int n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix Aux = I + (dt / 2) * A;
	Matrix Aux2 = I - (dt / 2) * A;

	Matrix newstates = Aux2.inverse()*Aux*states + Aux2.inverse()*(dt / 2) * B*(u_new + u_old) + Aux2.inverse()*dt*C;
	return newstates;
}

Matrix DPsim::Trapezoidal(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u)
{
	int n = states.rows();
	Matrix I = Matrix::Identity(n, n);

	Matrix Aux = I + (dt / 2) * A;
	Matrix Aux2 = I - (dt / 2) * A;

	Matrix newstates = Aux2.inverse()*Aux*states + Aux2.inverse()*dt*B*u + Aux2.inverse()*dt*C;
	return newstates;
}

Matrix DPsim::Trapezoidal(Matrix states, Matrix A, Matrix B, Real dt, Matrix u)
{
	int n = states.rows();

	Matrix I = Matrix::Identity(n, n);
	Matrix Aux = I + (dt / 2) * A;
	Matrix Aux2 = I - (dt / 2) * A;
	Matrix InvAux = Aux2.inverse();

	return InvAux*Aux*states + InvAux*dt*B*u;
}

Matrix DPsim::Euler(Matrix states, Matrix A, Matrix B, Real dt, Matrix u)
{
	return states + dt*(A*states + B*u);
}


Matrix DPsim::Euler(Matrix states, Matrix A, Matrix B, Matrix C, Real dt, Matrix u)
{

	Matrix newstates = states + dt*(A*states + B*u + C);
	return newstates;
}


