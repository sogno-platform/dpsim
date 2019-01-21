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

ODESolver::ODESolver(String name, CPS::ODEInterface::Ptr comp, bool implicit_integration, Real timestep)
	: mName(name), mComponent(comp), mImplicitIntegration(implicit_integration), mTimestep(timestep) {
	mProbDim = mComponent->attribute<Matrix>("ode_pre_state")->get().rows();
	initialize();
}

void ODESolver::initialize() {
	mStates=N_VNew_Serial(mProbDim);
	// Set initial value: (Different from DAESolver), only for already initialized components!
	// XXX
	N_VSetArrayPointer(mComponent->attribute<Matrix>("ode_post_state")->operator Matrix&().data(), mStates);
	// Forbid SUNdials from deleting the underlying state vector (which is managed
	// by our attribute / shared_ptr system)
	NV_OWN_DATA_S(mStates) = false;

	// Analogous to DAESolver
	CPS::ODEInterface::Ptr dummy = mComponent;
	mStSpFunction = [dummy](double t, const double y[], double ydot[]) {
		dummy->odeStateSpace(t, y, ydot);
	};
	mJacFunction = [dummy](double t, const double y[], double fy[], double J[],
	                       double tmp1[], double tmp2[], double tmp3[]) {
		dummy->odeJacobian(t, y, fy, J, tmp1, tmp2, tmp3);
	};


	// Causes numerical issues, better allocate in every step-> see step
	/*mArkode_mem= ARKodeCreate();
	 if (check_flag(mArkode_mem, "ARKodeCreate", 0))
		mFlag=1;

	mFlag = ARKodeSetUserData(mArkode_mem, this);
	if (check_flag(&mFlag, "ARKodeSetUserData", 1))
		mFlag=1;*/

		/* Call ARKodeInit to initialize the integrator memory and specify the
 	  right-hand side function in y'=f(t,y), the inital time T0, and
 	  the initial dependent variable vector y(fluxes+mech. vars).*/
 /*	if(mImplicitIntegration){
	 mFlag = ARKodeInit(mArkode_mem, NULL, &ODESolver::StateSpaceWrapper, initial_time, mStates);
	 if (check_flag(&mFlag, "ARKodeInit", 1)) throw CPS::Exception();

	 // Initialize dense matrix data structure
	 A = SUNDenseMatrix(mProbDim, mProbDim);
	 if (check_flag((void *)A, "SUNDenseMatrix", 0)) throw CPS::Exception();

	 // Initialize linear solver
	 LS = SUNDenseLinearSolver(mStates, A);
	 if (check_flag((void *)LS, "SUNDenseLinearSolver", 0)) throw CPS::Exception();

	 // Attach matrix and linear solver
	 mFlag = ARKDlsSetLinearSolver(mArkode_mem, LS, A);
	 if (check_flag(&mFlag, "ARKDlsSetLinearSolver", 1)) throw CPS::Exception();

	 // Set Jacobian routine
	 mFlag = ARKDlsSetJacFn(mArkode_mem, &ODESolver::JacobianWrapper);
	 if (check_flag(&mFlag, "ARKDlsSetJacFn", 1)) throw CPS::Exception();
 }
 else {
	 mFlag = ARKodeInit(mArkode_mem, &ODESolver::StateSpaceWrapper, NULL, initial_time, mStates);
	 if (check_flag(&mFlag, "ARKodeInit", 1)) throw CPS::Exception();
 }*/

	// Shifted to every step because of numerical issues

 	// Specify Runge-Kutta Method/order
 	/*mFlag = ARKodeSetOrder(mArkode_mem, 4);
 	if (check_flag(&mFlag, "ARKodeOrderSet", 1))
		mFlag=1;*/

	/*mFlag = ARKodeSStolerances(mArkode_mem, reltol, abstol);
	if (check_flag(&mFlag, "ARKodeSStolerances", 1))
		mFlag=1;*/
}

int ODESolver::StateSpaceWrapper(realtype t, N_Vector y, N_Vector ydot, void *user_data){
	ODESolver *self=reinterpret_cast<ODESolver *>(user_data);
	return self->StateSpace(t, y, ydot);
}

int ODESolver::StateSpace(realtype t, N_Vector y, N_Vector ydot){
	mStSpFunction(t, NV_DATA_S(y), NV_DATA_S(ydot));
	return 0;
}

int ODESolver::JacobianWrapper(realtype t, N_Vector y, N_Vector fy, SUNMatrix J, void *user_data,
               N_Vector tmp1, N_Vector tmp2, N_Vector tmp3){
	ODESolver *self=reinterpret_cast<ODESolver *>(user_data);
	return self->Jacobian(t, y, fy, J, tmp1, tmp2, tmp3);
}

int ODESolver::Jacobian(realtype t, N_Vector y, N_Vector fy, SUNMatrix J,
               N_Vector tmp1, N_Vector tmp2, N_Vector tmp3){
	mJacFunction(t, NV_DATA_S(y), NV_DATA_S(fy), SM_DATA_D(J),
	             NV_DATA_S(tmp1), NV_DATA_S(tmp2), NV_DATA_S(tmp3));
	return 0;
}

Real ODESolver::step(Real initial_time) {
	// Not absolutely necessary; realtype by default double (same as Real)
	realtype T0 = (realtype) initial_time;
	realtype Tf = (realtype) initial_time+mTimestep;

	/// Number of integration steps
	long int nst;
	/// Number of error test fails
	long int netf;

	mComponent->attribute<Matrix>("ode_post_state")->set(mComponent->attribute<Matrix>("ode_pre_state")->get());

	// Better allocate the arkode memory here to prevent numerical problems
	mArkode_mem= ARKodeCreate();
	 if (check_flag(mArkode_mem, "ARKodeCreate", 0))
		mFlag=1;

	mFlag = ARKodeSetUserData(mArkode_mem, this);
	if (check_flag(&mFlag, "ARKodeSetUserData", 1))
		mFlag=1;

		/* Call ARKodeInit to initialize the integrator memory and specify the
 	  right-hand side function in y'=f(t,y), the inital time T0, and
 	  the initial dependent variable vector y(fluxes+mech. vars).*/
 	if(mImplicitIntegration){
 		mFlag = ARKodeInit(mArkode_mem, NULL, &ODESolver::StateSpaceWrapper, initial_time, mStates);
 		if (check_flag(&mFlag, "ARKodeInit", 1)) throw CPS::Exception();

 		// Initialize dense matrix data structure
 	  A = SUNDenseMatrix(mProbDim, mProbDim);
 	  if (check_flag((void *)A, "SUNDenseMatrix", 0)) throw CPS::Exception();

 		// Initialize linear solver
 	  LS = SUNDenseLinearSolver(mStates, A);
 	  if (check_flag((void *)LS, "SUNDenseLinearSolver", 0)) throw CPS::Exception();

 		// Attach matrix and linear solver
 		mFlag = ARKDlsSetLinearSolver(mArkode_mem, LS, A);
 		if (check_flag(&mFlag, "ARKDlsSetLinearSolver", 1)) throw CPS::Exception();

 		// Set Jacobian routine
 		mFlag = ARKDlsSetJacFn(mArkode_mem, &ODESolver::JacobianWrapper);
 		if (check_flag(&mFlag, "ARKDlsSetJacFn", 1)) throw CPS::Exception();
 	}
 	else {
 		mFlag = ARKodeInit(mArkode_mem, &ODESolver::StateSpaceWrapper, NULL, initial_time, mStates);
 		if (check_flag(&mFlag, "ARKodeInit", 1)) throw CPS::Exception();
 	}

	mFlag = ARKodeSStolerances(mArkode_mem, reltol, abstol);
	if (check_flag(&mFlag, "ARKodeSStolerances", 1))
		mFlag=1;


	// Main integrator loop
	realtype t = T0;
	while (Tf-t > 1.0e-15) {
		mFlag = ARKode(mArkode_mem, Tf, mStates, &t, ARK_NORMAL);
		if (check_flag(&mFlag, "ARKode", 1))	break;
	}

	// Get some statistics to check for numerical problems (instability, blow-up etc)
	mFlag = ARKodeGetNumSteps(mArkode_mem, &nst);
	 if(check_flag(&mFlag, "ARKodeGetNumSteps", 1))
	 	return 1;
	mFlag = ARKodeGetNumErrTestFails(mArkode_mem, &netf);
	if(check_flag(&mFlag, "ARKodeGetNumErrTestFails", 1))
		return 1;

	// Print statistics:
	std::cout << "Number Computing Steps: "<< nst << " Number Error-Test-Fails: " << netf << std::endl;
	return Tf;
}

void ODESolver::SolveTask::execute(Real time, Int timeStepCount) {
	mSolver.step(time);
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
  SUNLinSolFree(LS);
	SUNMatDestroy(A);
}
