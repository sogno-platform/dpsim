/** ODE Solver
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/ODESolver.h>
#include <cps/PowerComponent.h>

using namespace DPsim;
//using namespace CPS;

ODESolver::ODESolver(String name, CPS::ODEInterface::Ptr comp, Real dt, Real t0):
	mTimestep(dt){

	mComponent=comp;
	mProbDim=mComponent->num_states();
	initialize(t0);
} //Probably issue with constructor

void ODESolver::initialize(Real t0) {

	// Component initialization needed?
	mStates=N_VNew_Serial(mProbDim);
	// Set initial value: (Different from DAESolver)
	N_VSetArrayPointer(mComponent->state_vector(),mStates); //breaks down
	/*std::cout<< NV_Ith_S(mStates,0)<< " "<<  NV_Ith_S(mStates,1)<< " "<<NV_Ith_S(mStates,2)<< " "<<NV_Ith_S(mStates,3)<< " "<<NV_Ith_S(mStates,4)
	<< " "<<NV_Ith_S(mStates,5)<< " "<<NV_Ith_S(mStates,6)<< " "<<NV_Ith_S(mStates,7)<< " "<<NV_Ith_S(mStates,8)<< std::endl;*/
	// Copied from DAESolver
  CPS::ODEInterface::Ptr dummy = mComponent;
	mStSpFunction=[dummy](double t, const double  y[], double  ydot[]){
		dummy->StateSpace(t, y, ydot);};

	mArkode_mem= ARKodeCreate();
	 if (check_flag(mArkode_mem, "ARKodeCreate", 0))
		mFlag=1;

	mFlag = ARKodeSetUserData(mArkode_mem, this);
	if (check_flag(&mFlag, "ARKodeSetUserData", 1))
		mFlag=1;

		// Call ARKodeInit to initialize the integrator memory and specify the
 	 // right-hand side function in y'=f(t,y), the inital time T0, and
 	 // the initial dependent variable vector y(fluxes+mech. vars).
 /*	if(implicit) {
 		flag = ARKodeInit(arkode_mem, NULL, &ODESolver::StateSpaceWrapper, T0, y);
 		if (check_flag(&flag, "ARKodeInit", 1)) throw CPS::Exception();

 		// Initialize dense matrix data structure
 	  A = SUNDenseMatrix(dim, dim);
 	  if (check_flag((void *)A, "SUNDenseMatrix", 0)) throw CPS::Exception();

 		// Initialize linear solver
 	  LS = SUNDenseLinearSolver(y, A);
 	  if (check_flag((void *)LS, "SUNDenseLinearSolver", 0)) throw CPS::Exception();

 		// Attach matrix and linear solver
 		flag = ARKDlsSetLinearSolver(arkode_mem, LS, A);
 		if (check_flag(&flag, "ARKDlsSetLinearSolver", 1)) throw CPS::Exception();

 		// Set Jacobian routine
 		flag = ARKDlsSetJacFn(arkode_mem, &ODESolver::JacobianWrapper);
 		if (check_flag(&flag, "ARKDlsSetJacFn", 1)) throw CPS::Exception();
 	}
 	else {
 		flag = ARKodeInit(arkode_mem, &ODESolver::StateSpaceWrapper, NULL, T0, y);
 		if (check_flag(&flag, "ARKodeInit", 1)) throw CPS::Exception();
 	}*/

 	// Specify Runge-Kutta Method/order
 	//flag = ARKodeSetOrder(arkode_mem, 4);
 	//if (check_flag(&flag, "ARKodeOrderSet", 1)) return 1;

	mFlag = ARKodeInit(mArkode_mem, &ODESolver::StateSpaceWrapper, NULL, t0, mStates);
	if (check_flag(&mFlag, "ARKodeInit", 1))
		mFlag=1;

	mFlag = ARKodeSStolerances(mArkode_mem, reltol, abstol);
	if (check_flag(&mFlag, "ARKodeSStolerances", 1))
		mFlag=1;
}

int ODESolver::StateSpaceWrapper(realtype t, N_Vector y, N_Vector ydot, void *user_data){
	ODESolver *self=reinterpret_cast<ODESolver *>(user_data);
	return self->StateSpace(t, y, ydot);
}
/*
int ODESolver::JacobianWrapper(realtype t, N_Vector y, N_Vector fy, SUNMatrix J, void *user_data,
               N_Vector tmp1, N_Vector tmp2, N_Vector tmp3){
	ODESolver *self=reinterpret_cast<ODESolver *>(user_data);
	return self->Jacobian(t, y, fy, J, tmp1, tmp2, tmp3);
}*/

int ODESolver::StateSpace(realtype t, N_Vector y, N_Vector ydot){
	mStSpFunction(t, NV_DATA_S(y), NV_DATA_S(ydot));
	return 0;
}

Real ODESolver::step(Real initial_time) {
	// Not absolutely necessary; realtype by default double (same as Real)
	realtype T0 = (realtype) initial_time;
	realtype Tf = (realtype) initial_time+mTimestep;

	// number integration steps
	//long int nst,
	// number error test fails
	//long int netf;

	// Main integrator loop
	realtype t = T0;
	while (Tf-t > 1.0e-15) {
		mFlag = ARKode(mArkode_mem, Tf, mStates, &t, ARK_NORMAL);
		if (check_flag(&mFlag, "ARKode", 1))	break;
	}

	return Tf;
}

// ARKode-Error checking functions
// Check function return value...
//	opt == 0 means SUNDIALS function allocates memory so check if
//			 returned NULL pointer
//	opt == 1 means SUNDIALS function returns a flag so check if
//			 flag >= 0
//	opt == 2 means function allocates memory so check if returned
//			 NULL pointer
int ODESolver::check_flag(void *flagvalue, const std::string funcname, int opt){
  int *errflag;

// Check if SUNDIALS function returned NULL pointer - no memory allocated
  if (opt == 0 && flagvalue == NULL) {
    std::cout << "\nSUNDIALS_ERROR: " << funcname << " failed - returned NULL pointer\n\n";
    return 1; }

  // Check if flag < 0
  else if (opt == 1) {
    errflag = (int *) flagvalue;
    if (*errflag < 0) {
      std::cout << "\nSUNDIALS_ERROR: " << funcname << " failed with flag = " << *errflag << "\n\n";
      return 1;
    }
  }

  // Check if function returned NULL pointer - no memory allocated
  else if (opt == 2 && flagvalue == NULL) {
    std::cout << "\nMEMORY_ERROR: " << funcname << " failed - returned NULL pointer\n\n";
    return 1; }

  return 0;
}

ODESolver::~ODESolver() {
	ARKodeFree(&mArkode_mem);
	N_VDestroy(mStates);
  //SUNLinSolFree(LS);
	//SUNMatDestroy(A);
}
