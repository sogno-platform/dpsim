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
#include <cps/Solver/MNAInterface.h>

using namespace DPsim;
using namespace CPS;

//#define NVECTOR_DATA(vec) NV_DATA_S (vec) // Returns pointer to the first element of array vec

DAESolver::DAESolver(String name, CPS::SystemTopology system, Real dt, Real t0) :
        mSystem(system),
        mTimestep(dt) {

    // Defines offset vector of the residual which is composed as follows:
    // mOffset[0] = # nodal voltage equations
    // mOffset[1] = # of components and their respective equations (1 per component for now as inductance is not yet considered)

    mOffsets.push_back(0);
    mOffsets.push_back(0);

    // Set initial values of all required variables and create IDA solver environment
    for(Component::Ptr comp : mSystem.mComponents) {
        auto daeComp = std::dynamic_pointer_cast<DAEInterface>(comp);
        std::cout <<"Added Comp"<<std::endl;
        if (!daeComp)
            throw CPS::Exception(); // Component does not support the DAE solver interface

        mComponents.push_back(comp);
    }

    for (auto baseNode : mSystem.mNodes) {
        // Add nodes to the list and ignore ground nodes.
        if (!baseNode->isGround()) {
            auto node = std::dynamic_pointer_cast<CPS::Node<Complex>>(baseNode);
            mNodes.push_back(node);
            std::cout <<"Added Node"<<std::endl;
        }
    }

    mNEQ = mComponents.size() + (2 * mNodes.size());
    std::cout<<std::endl;
    std::cout<<"Number of Eqn. "<<mNEQ<<std::endl;

    std::cout <<"Processing Nodes"<<std::endl;
    UInt simNodeIdx = 0;

    for (UInt idx = 0; idx < mNodes.size(); idx++) {

        mNodes[idx]->setSimNode(0, simNodeIdx);
        simNodeIdx++;
        if (mNodes[idx]->phaseType() == PhaseType::ABC) {
            mNodes[idx]->setSimNode(1, simNodeIdx);
            simNodeIdx++;
            mNodes[idx]->setSimNode(2, simNodeIdx);
            simNodeIdx++;
        }
    }

    std::cout <<"Nodes Setup Done"<<std::endl;
    initialize(t0);
}

void DAESolver::initialize(Real t0) {
    int ret;
    int counter = 0;
    realtype *sval = NULL, *s_dtval = NULL;
    std::cout << "Init states" << std::endl;

    // Allocate state vectors
    state = N_VNew_Serial(mNEQ);
    dstate_dt = N_VNew_Serial(mNEQ);

    std::cout << "State Init done" << std::endl;
    std::cout << "Pointer Init" << std::endl;
    sval = N_VGetArrayPointer_Serial(state);
    s_dtval = N_VGetArrayPointer_Serial(dstate_dt);
    std::cout << "Pointer Init done" << std::endl << std::endl;

    for (auto node : mNodes) {
        // Initialize nodal voltages of state vector
        Real tempVolt;
        PhaseType phase = node->phaseType();
        tempVolt = std::real(node->initialSingleVoltage(phase));

        std::cout << "Node Volt " << counter << ": " << tempVolt << std::endl;
        sval[counter++] = tempVolt;
    }



    for (Component::Ptr comp : mComponents) {
        auto emtComp = std::dynamic_pointer_cast<PowerComponent<Complex> >(comp);
        if (emtComp) {
            emtComp->initializeFromPowerflow(mSystem.mSystemFrequency);// Set initial values of all components
        }

        auto daeComp = std::dynamic_pointer_cast<DAEInterface>(comp);
        if (!daeComp)
            throw CPS::Exception();

        // Initialize component values of state vector
        sval[counter++] = std::real(daeComp->daeInitialize());
        std::cout << "Comp Volt " << counter-1 << ": " << sval[counter-1]<< std::endl;
//		sval[counter++] = component inductance;


        // Register residual functions of components


        mResidualFunctions.push_back(
                [daeComp](double ttime, const double state[], const double dstate_dt[], double resid[],
                          std::vector<int> &off) {
                    daeComp->daeResidual(ttime, state, dstate_dt, resid, off);
                });
    }

    for (int j = 0; j < (int) mNodes.size(); j++) {
        // Initialize nodal current equations
        sval[counter++] = 0; //TODO: check for correctness
        std::cout << "Nodal Equation value " << sval[counter - 1] << std::endl;
    }

    std::cout << std::endl;

    // Set initial values for state derivative for now all equal to 0
    for (int i = 0; i < (mNEQ); i++) {
        s_dtval[i] = 0; // TODO: add derivative calculation
        std::cout << "derivative " << i << ": " << s_dtval[i] << std::endl;
    }

    std::cout << std::endl;

    std::cout << "Init Tolerances" << std::endl;
    rtol = RCONST(1.0e-10); // Set relative tolerance
    abstol = RCONST(1.0e-4); // Set absolute error
    std::cout << "Init Tolerances done" << std::endl;

    mem = IDACreate();
    if (mem != NULL) std::cout << "Memory Ok" << std::endl;
    std::cout << "Define Userdata" << std::endl;
    // This passes the solver instance as the user_data argument to the residual functions
    ret = IDASetUserData(mem, this);
//	if (check_flag(&ret, "IDASetUserData", 1)) {
//		throw CPS::Exception();
//	}
    std::cout << "Call IDAInit" << std::endl;

    ret = IDAInit(mem, &DAESolver::residualFunctionWrapper, t0, state, dstate_dt);
//	if (check_flag(&ret, "IDAInit", 1)) {
//		throw CPS::Exception();
//	}
    std::cout << "Call IDATolerances" << std::endl;
    ret = IDASStolerances(mem, rtol, abstol);
// 	if (check_flag(&ret, "IDASStolerances", 1)) {
//		throw CPS::Exception();
//	}
    std::cout << "Call IDA Solver Stuff" << std::endl;
    // Allocate and connect Matrix A and solver LS to IDA
    A = SUNDenseMatrix(mNEQ, mNEQ);
    LS = SUNDenseLinearSolver(state, A);
    ret = IDADlsSetLinearSolver(mem, LS, A);

    //Optional IDA input functions
    //ret = IDASetMaxNumSteps(mem, -1);  //Max. number of timesteps until tout (-1 = unlimited)
    //ret = IDASetMaxConvFails(mem, 100); //Max. number of convergence failures at one step
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
    double *residual = NV_DATA_S(resid);
    double *tempstate = NV_DATA_S(state);
    // Solve for all node Voltages
    for (auto node : mNodes) {

        Real tempVolt = 0;

        tempVolt += std::real(node->singleVoltage());


        residual[mOffsets[0]] = tempVolt - tempstate[mOffsets[0]];
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

    Real NextTime = time + mTimestep;
    std::cout<<"Current Time "<<NextTime<<std::endl;
    int ret = IDASolve(mem, NextTime, &tret, state, dstate_dt, IDA_NORMAL);  // TODO: find alternative to IDA_NORMAL

    if (ret == IDA_SUCCESS) {
        return NextTime;
    }
    else {
        std::cout <<"Ida Error "<<ret<<std::endl;
        //throw CPS::Exception();
        void(IDAGetNumSteps(mem, &interalSteps));
        void(IDAGetNumResEvals(mem, &resEval));
        std::cout<<"Interal steps: "<< interalSteps<<std::endl;
        std::cout<<"Res Eval :"<<resEval<<std::endl;
        return NextTime;
    }
}

Task::List DAESolver::getTasks() {
	// TODO
	return Task::List();
}

DAESolver::~DAESolver() {
    // Releasing all memory allocated by IDA
    IDAFree(&mem);
    N_VDestroy(state);
    N_VDestroy(dstate_dt);
    SUNLinSolFree(LS);
    SUNMatDestroy(A);
}
