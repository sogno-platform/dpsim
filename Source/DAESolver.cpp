/** DAE Solver
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

#include <dpsim/DAESolver.h>
#include <cps/PowerComponent.h>

using namespace DPsim;
using namespace CPS;

//#define NVECTOR_DATA(vec) NV_DATA_S (vec) // Returns pointer to the first element of array vec

DAESolver::DAESolver(String name, SystemTopology system, Real dt, Real t0) :
	mSystem(system),
	mTimestep(dt) {

	// Defines offset vector of the residual which is composed as follows:
	// mOffset[0] = # nodal voltage equations
	// mOffset[1] = # of componets and their respective equations (1 per component for now as inductance is not yet considered)
	mOffsets.push_back(0);
	mOffsets.push_back(0);
	mNEQ = mSystem.mComponents.size() + (2 * mSystem.mNodes.size());

	// Set inital values of all required variables and create IDA solver environment
	for(Component::Ptr comp : mSystem.mComponents) {
		auto daeComp = std::dynamic_pointer_cast<DAEInterface>(comp);
		if (!daeComp)
			throw CPS::Exception(); // Commponent does not support the DAE solver interface

		mComponents.push_back(comp);
	}

	for (auto baseNode : mSystem.mNodes) {
		// Add nodes to the list and ignore ground nodes.
		if (!baseNode->isGround()) {
			auto node = std::dynamic_pointer_cast<Node<Real> >(baseNode);
			mNodes.push_back(node);
		}
	}

	UInt simNodeIdx = -1;
	for (UInt idx = 0; idx < mNodes.size(); idx++) {
		mNodes[idx]->simNodes()[0] = ++simNodeIdx;
		if (mNodes[idx]->phaseType() == PhaseType::ABC) {
			mNodes[idx]->simNodes()[1] = ++simNodeIdx;
			mNodes[idx]->simNodes()[2] = ++simNodeIdx;
		}
	}

	initialize(t0);
}

void DAESolver::initialize(Real t0)
{
	int ret;

	// Set inital values of all components
	for (Component::Ptr comp : mComponents) {
		auto emtComp = std::dynamic_pointer_cast<PowerComponent<Real> >(comp);
		if (emtComp)
			emtComp->initializeFromPowerflow(mSystem.mSystemFrequency);
	}

	// Allocate state vectors
	state = N_VNew_Serial(mNEQ);
	dstate_dt = N_VNew_Serial(mNEQ);

//	avtol = N_VNew_Serial(mNEQ);

	realtype *sval = NULL, *s_dtval=NULL ;

	sval = N_VGetArrayPointer_Serial(state);

	int counter = 0;
	for (auto node : mNodes) {
		// Initialize nodal values of state vector
		Real tempVolt = 0;
		tempVolt += std::real(node->initialVoltage()(0,0));
		sval[counter++] = tempVolt;

//		if (node->phaseType() == PhaseType::ABC) {
//			tempVolt += std::real(node->initialVoltage()(1,0));
//			tempVolt += std::real(node->initialVoltage()(2,0));
//		}
	}

	for (Component::Ptr comp : mComponents) {
		auto emtComp = std::dynamic_pointer_cast<PowerComponent<Real> >(comp);
		if (!emtComp)
			throw CPS::Exception();

		// Initialize component values of state vector
		sval[counter++] = emtComp->voltage();
//		sval[counter++] = component inductance;

		// Register residual functions of components
		auto daeComp = std::dynamic_pointer_cast<DAEInterface>(comp);
		if (!daeComp)
			throw CPS::Exception();

		mResidualFunctions.push_back([daeComp](double ttime, const double state[], const double dstate_dt[], double resid[], std::vector<int>& off) {
			daeComp->daeResidual(ttime, state, dstate_dt, resid, off);
		});
	}

	for (int j = 1; j < mNEQ; j++) {
		// Initialize nodal current equations
		sval[counter++] = 0;
	}

	s_dtval = N_VGetArrayPointer_Serial(dstate_dt);

	// Set inital values for state derivative for now all equal to 0
	for (int i = 0; i < (mNEQ-1); i++) {
		s_dtval[i] = 0; // TODO: add derivative calculation
	}

	rtol = RCONST(1.0e-6); // Set relative tolerance
	abstol = RCONST(1.0e-1); // Set absolute error

	mem = IDACreate();

	// This passes the solver instance as the user_data argument to the residual functions
	ret = IDASetUserData(mem, this);
//	if (check_flag(&ret, "IDASetUserData", 1)) {
//		throw CPS::Exception();
//	}

	ret = IDAInit(mem, &DAESolver::residualFunctionWrapper, t0, state, dstate_dt);
//	if (check_flag(&ret, "IDAInit", 1)) {
//		throw CPS::Exception();
//	}

	ret = IDASStolerances(mem, rtol, abstol);
// 	if (check_flag(&ret, "IDASStolerances", 1)) {
//		throw CPS::Exception();
//	}

	// Allocate and connect Matrix A and solver LS to IDA
	A = SUNDenseMatrix(mNEQ, mNEQ);
	LS = SUNDenseLinearSolver(state, A);
	ret = IDADlsSetLinearSolver(mem, LS, A);

	(void) ret;
}

int DAESolver::residualFunctionWrapper(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data)
{
	DAESolver *self = reinterpret_cast<DAESolver *>(user_data);

	return self->residualFunction(ttime, state, dstate_dt, resid);
}

int DAESolver::residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid)
{
	mOffsets[0] = 0; // Reset Offset
	mOffsets[1] = 0; // Reset Offset

	// Solve for all node Voltages
	for (auto node : mNodes) {
		double *residual = NV_DATA_S(resid);
		double *tempstate = NV_DATA_S(state);
		Real tempVolt = 0;

		tempVolt += std::real(node->voltage()(0,0));

//		if (node->phaseType() == PhaseType::ABC) {
//			tempVolt += std::real(node->voltage()(1,0));
//			tempVolt += std::real(node->voltage()(2,0));
//			sval[counter++] = tempVolt;
//		}

		residual[mOffsets[0]] = tempstate[mOffsets[0]] - tempVolt;
		mOffsets[0] += 1;
	}

	// Call all registered component residual functions
	for (auto resFn : mResidualFunctions) {
		resFn(ttime, NV_DATA_S(state), NV_DATA_S(dstate_dt), NV_DATA_S(resid), mOffsets);
	}

	// If successful; positive value if recoverable error, negative if fatal error
	// TODO: Error handling
	return 0;
}

Real DAESolver::step(Real time) {

	int ret = IDASolve(mem, time, &tret, state, dstate_dt, IDA_NORMAL);  // TODO: find alternative to IDA_NORMAL

	if (ret == IDA_SUCCESS) {
		return time + mTimestep	;
	}
	else {
		std::cout <<"Ida Error"<<std::endl;
		return time + mTimestep	;
	}
}

DAESolver::~DAESolver() {
	// Releasing all memory allocated by IDA
	IDAFree(&mem);
	N_VDestroy(state);
	N_VDestroy(dstate_dt);
	SUNLinSolFree(LS);
	SUNMatDestroy(A);
}
