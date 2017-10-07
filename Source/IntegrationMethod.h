#ifndef INTEGRATIONMETHOD_H
#define INTEGRATIONMETHOD_H

#include "MathLibrary.h"

DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u_new, DPSMatrix u_old);
DPSMatrix Trapezoidal(DPSMatrix states, DPSMatrix A, DPSMatrix B, double dt, DPSMatrix u);

#endif