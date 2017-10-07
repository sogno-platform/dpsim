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

DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u)
{
	int n = states.rows();

	DPSMatrix I = DPSMatrix::Identity(n, n);
	DPSMatrix Aux = I + (dt / 2) * A;
	DPSMatrix Aux2 = I - (dt / 2) * A;
	DPSMatrix InvAux = Aux2.inverse();

	return InvAux*Aux*states + InvAux*dt*B*u;
}