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

DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u_new, DPSMatrix u_old)
{
	int n = states.rows();
	DPSMatrix I = DPSMatrix::Identity(n, n);

	DPSMatrix Aux = I + (dt / 2) * A;
	DPSMatrix Aux2 = I - (dt / 2) * A;

	DPSMatrix newstates = Aux2.inverse()*Aux*states + Aux2.inverse()*(dt / 2) * B*(u_new + u_old);
	return newstates;
}

DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, DPSMatrix C, double dt, DPSMatrix u_new, DPSMatrix u_old)
{
	int n = states.rows();
	DPSMatrix I = DPSMatrix::Identity(n, n);

	DPSMatrix Aux = I + (dt / 2) * A;
	DPSMatrix Aux2 = I - (dt / 2) * A;

	DPSMatrix newstates = Aux2.inverse()*Aux*states + Aux2.inverse()*(dt / 2) * B*(u_new + u_old) + Aux2.inverse()*dt*C;
	return newstates;
}

DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u)
{
	int n = states.rows();

	DPSMatrix I = DPSMatrix::Identity(n, n);
	DPSMatrix Aux = I + (dt / 2) * A;
	DPSMatrix Aux2 = I - (dt / 2) * A;
	DPSMatrix InvAux = Aux2.inverse();

	return InvAux*Aux*states + InvAux*dt*B*u;
}

DPSMatrix Euler(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u)
{
	return states + dt*(A*states + B*u);
}

double Euler(double state, DPSMatrix inputs, DeriveFnPtr fnPtr, double dt) {
	return state + dt*fnPtr(inputs);
}

DPSMatrix Euler(DPSMatrix states, DPSMatrix A, DPSMatrix B, DPSMatrix C, double dt, DPSMatrix u)
{

	DPSMatrix newstates = states + dt*(A*states + B*u + C);
	return newstates;
}


