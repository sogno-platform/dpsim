/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/DAESolver.h>

using namespace DPsim;
using namespace CPS;

static int check_retval(void *returnvalue, const char *funcname, int opt);
static void get_error_str(int error_code, int IDAMaxNumSteps, int IDAMaxErrTestFails, int IDAMaxConvFails, std::string& ret);
//static void PrintOutput(void *mem, realtype t, N_Vector y);

template <typename VarType>
DAESolver<VarType>::DAESolver(const String& name, CPS::SystemTopology system, 
    Real dt, Real t0, CPS::Logger::Level logLevel) :
	Solver(name, logLevel), mSystem(system), mTimestep(dt) {

    mSimTime = t0;
    mInitTime = t0;

    // Defines offset vector of the residual which is composed as follows:
    // mOffset[0] = # nodal voltage equations
    // mOffset[1] = # of components and their respective equations (1 per component for now as inductance is not yet considered)

    mOffsets.push_back(0);
    mOffsets.push_back(0);

    // Set number of equations to zero
    mNEQ=0;

    //
    mSLog->info("-- Process system components");
    this->initializeComponents();

    //
    mSLog->info("-- Process system nodes");
    for (auto baseNode : mSystem.mNodes) {
        // Add nodes to the list and ignore ground nodes.
        if (!baseNode->isGround()) {
            auto node = std::dynamic_pointer_cast<CPS::SimNode<VarType>>(baseNode);
            if (!node) 
                throw CPS::Exception(); 
            
            if (node->phaseType() == PhaseType::Single) {
                mNEQ += 1;
            } else if (node->phaseType() == PhaseType::ABC) {
                mNEQ += 3;
            }
            mNodes.push_back(node);
            mSLog->info("Added node {:s} to state vector", node->name());
        }
    }

    UInt matrixNodeIndex = 0;
    for (UInt idx = 0; idx < mNodes.size(); idx++) {
        mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndex);
        ++matrixNodeIndex;
        if (mNodes[idx]->phaseType() == PhaseType::ABC) {
            mNodes[idx]->setMatrixNodeIndex(1, matrixNodeIndex);
            ++matrixNodeIndex;
            mNodes[idx]->setMatrixNodeIndex(2, matrixNodeIndex);
            ++matrixNodeIndex;
        }
    }
    mSLog->info("");
    mSLog->flush();
}


template <typename VarType>
void DAESolver<VarType>::initializeComponents() {
    mSLog->info("-- Initialize components from power flow");
    for(IdentifiedObject::Ptr comp : mSystem.mComponents) {
        // Initialize components and add component eqs. to state vector
        auto emtComp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
        if (!emtComp) {
            throw CPS::Exception();
        }
        // Set initial values of all components
        emtComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
        auto daeComp = std::dynamic_pointer_cast<DAEInterface>(comp);
        if (!daeComp) {
            throw CPS::Exception();
        }

        mDAEComponents.push_back(daeComp);

        // Register residual functions of components
        mResidualFunctions.push_back(
                [daeComp](double sim_time, const double state[], const double dstate_dt[], 
                            double resid[], std::vector<int> &off) {
                                daeComp->daeResidual(sim_time, state, dstate_dt, resid, off);
                            });

        // Register jacobian functions of components
        mJacobianFunctions.push_back(
                [daeComp](double current_time, const double state[], const double dstate_dt[], 
			                SUNMatrix jacobian, double cj, std::vector<int>& off) {
                                daeComp->daeJacobian(current_time, state, dstate_dt, jacobian, cj, off);
                            });

        mNEQ += daeComp->getNumberOfStateVariables();
        mSLog->info("Added {:s} '{:s}' to simulation.", comp->type(), comp->name());
        mSLog->flush();
    }
}

template <typename VarType>
void DAESolver<VarType>::initialize() {
    mSLog->info("---- Start DAE initialization ----");
    mSLog->info("Number of Eqn.: {}", mNEQ);
    
    /* Create SUNDIALS context */
    auto ret = SUNContext_Create(NULL, &mSunctx);
    if (check_retval(&ret, "SUNContext_Create", 1)) throw CPS::Exception();

    int counter = 0;
    realtype *sval = NULL;
    realtype *s_dtval = NULL;
    realtype *state_var_types = NULL;
    realtype *absolute_tolerances = NULL;
    
    // creates and allocates memory for state vector
    mStateVector = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mStateVector, "N_VNew_Serial", 0)) throw CPS::Exception();
    
    // creates and allocates memory for derivatives of state vector
    mDerivativeStateVector = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mDerivativeStateVector, "N_VNew_Serial", 0)) throw CPS::Exception();

    // creates and allocates memory for vector of variable types (differential or algebraic)
    mStateIDsVector = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mStateIDsVector, "N_VNew_Serial", 0)) throw CPS::Exception();

    // creates and allocates memory for vector of absolute tolerances
    mAbsoluteTolerances = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mAbsoluteTolerances, "N_VNew_Serial", 0)) throw CPS::Exception();

    // creates and allocates memory for vector of error weights
    mErrorWeights = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mErrorWeights, "N_VNew_Serial", 0)) throw CPS::Exception();
    
    // creates and allocates memory for vector of estimated local errors
    mEstLocalErrors = N_VNew_Serial(mNEQ, mSunctx);
    if(check_retval((void *)mEstLocalErrors, "N_VNew_Serial", 0)) throw CPS::Exception();

    // capturing a returned array/pointer
    sval  = N_VGetArrayPointer(mStateVector);
    s_dtval = N_VGetArrayPointer_Serial(mDerivativeStateVector);
    state_var_types = N_VGetArrayPointer_Serial(mStateIDsVector);
    absolute_tolerances = N_VGetArrayPointer_Serial(mAbsoluteTolerances);

    // Initialize nodal voltages of state vector
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
	        Real omega = 2 * PI * mSystem.mSystemFrequency;
            Complex initVoltage = node->initialSingleVoltage(PhaseType::Single);
            sval[counter] = std::real(node->initialSingleVoltage(PhaseType::Single));
            s_dtval[counter] = -omega * initVoltage.imag();
            node->setVoltage(sval[counter], PhaseType::Single);
            node->setdVoltage(s_dtval[counter]);
            state_var_types[counter] = (Real)(0.0);	     //set node variable as algebraic variable
            absolute_tolerances[counter] = (Real)(node->daeGetAbsoluteTolerance()); 
            mSLog->info(
		        "Added node '{:s}' to state vector, init voltage = {:f}V"
                "\nAdded derivative of the voltage node of '{:s}' to derivative state vector, initial value = {:f}"
                "\nAbsolute_tolerances of node '{:s}' = {:f}",
                node->name(), sval[counter], node->name(), s_dtval[counter], node->name(), absolute_tolerances[counter]
            );
            counter++;
        }
        else if (node->phaseType() == PhaseType::ABC) {
            Real frequency = mSystem.mSystemFrequency;
	        Real omega = 2 * PI * frequency;
           
            Complex initVolt_phase_A = RMS3PH_TO_PEAK1PH * node->initialSingleVoltage(PhaseType::A);
            Complex initVolt_phase_B = RMS3PH_TO_PEAK1PH * node->initialSingleVoltage(PhaseType::B);
            Complex initVolt_phase_C = RMS3PH_TO_PEAK1PH * node->initialSingleVoltage(PhaseType::C);

            sval[counter] = std::real(initVolt_phase_A);
            s_dtval[counter] = -omega*initVolt_phase_A.imag();
            node->setVoltage(sval[counter],PhaseType::A);
            node->setdVoltage(s_dtval[counter], PhaseType::A);
            state_var_types[counter] = (Real)(0.0);	        //set node variable as differential variable
            absolute_tolerances[counter] = (Real)(node->daeGetAbsoluteTolerance()); ;    
            mSLog->info(
		        "Added node '{:s}'-phase_A to state vector, init voltage = {:f}V"
                "\nAdded derivative of the voltage node of '{:s}'-phase_A to derivative state vector, initial value = {:f}"
                "\nAbsolute_tolerances = {:f}",
                node->name(), sval[counter], node->name(), s_dtval[counter], absolute_tolerances[counter]
            );
            counter++;

            sval[counter] = std::real(initVolt_phase_B);
            s_dtval[counter] = -omega*initVolt_phase_B.imag();
            node->setVoltage(sval[counter],PhaseType::B);
            node->setdVoltage(s_dtval[counter], PhaseType::B);
            state_var_types[counter] = (Real)(0.0);	        //set node variable as differential variable
            absolute_tolerances[counter] = (Real)(node->daeGetAbsoluteTolerance()); 
            mSLog->info(
		        "Added node '{:s}'-phase_B to state vector, init voltage = {:f}V"
                "\nAdded derivative of the voltage node of '{:s}'-phase_B to derivative state vector, initial value = {:f}"
                "\nAbsolute_tolerances = {:f}",
                node->name(), sval[counter], node->name(), s_dtval[counter], absolute_tolerances[counter]
            );
            counter++;

            sval[counter] = std::real(initVolt_phase_C);
            s_dtval[counter] = -omega*initVolt_phase_C.imag();
            node->setVoltage(sval[counter],PhaseType::C);
            node->setdVoltage(s_dtval[counter], PhaseType::C);
            state_var_types[counter] = (Real)(0.0);	        //set node variable as differential variable
            absolute_tolerances[counter] = (Real)(node->daeGetAbsoluteTolerance()); 
            mSLog->info(
		        "Added node '{:s}'-phase_C to state vector, init voltage = {:f}V"
                "\nAdded derivative of the voltage node of '{:s}'-phase_C to derivative state vector, initial value = {:f}"
                "\nAbsolute_tolerances of node = {:f}",
                node->name(), sval[counter], node->name(), s_dtval[counter], absolute_tolerances[counter]
            );
            counter++;
        }
    }

    for (auto daeComp : mDAEComponents) {
        // Initialize components of state vector
        daeComp->daeInitialize(mInitTime, sval, s_dtval, 
            absolute_tolerances, state_var_types, counter);
    }

    // creates the IDA solver memory block
    mSLog->info("");
    mSLog->info("Creates the IDA solver memory block");
    mIDAMemoryBlock = IDACreate(mSunctx);
    if (mIDAMemoryBlock == NULL) throw CPS::Exception(); 
    
    // This passes the solver instance as the user_data argument to the residual functions
    mSLog->info("Define Userdata");
    ret = IDASetUserData(mIDAMemoryBlock, this);
	if (check_retval(&ret, "IDASetUserData", 1)) throw CPS::Exception();

    //
    mSLog->info("Call IDAInit");
    ret = IDAInit(mIDAMemoryBlock, &DAESolver::residualFunctionWrapper, mInitTime, 
        mStateVector, mDerivativeStateVector);
    if (check_retval(&ret, "IDAInit", 1)) throw CPS::Exception();

    // Set relative and absolute tolerances
    mSLog->info("Call IDATolerances");
    ret = IDASVtolerances(mIDAMemoryBlock, mRelativeTolerance, mAbsoluteTolerances);
    if (check_retval(&ret, "IDASVtolerances", 1)) throw CPS::Exception();
        mSLog->info("Set relative tolerance =  {:f}", mRelativeTolerance);

    // ### Nonlinear solver interface optional input functions ### 
    ///  specifies the maximum number of nonlinear solver iterations in one solve attempt
    mSLog->info("Call IDASetMaxNonlinIters");
    ret = IDASetMaxNonlinIters(mIDAMemoryBlock, mIDAMaxNonlinIters);
    if (check_retval(&ret, "IDASetMaxNonlinIters", 1)) throw CPS::Exception();
    mSLog->info("MaxNonlinIters set to {}", mIDAMaxNonlinIters);

    /// specifies the maximum number of nonlinear solver convergence failures in one step
    mSLog->info("Call IDASetMaxConvFails");
    ret = IDASetMaxConvFails(mIDAMemoryBlock, mIDAMaxConvFails);
    if (check_retval(&ret, "IDASetMaxConvFails", 1)) throw CPS::Exception();
    mSLog->info("MaxConvFails set to {}", mIDAMaxConvFails);

    /// specifies the safety factor in the nonlinear convergence test
    mSLog->info("Call IDASetNonlinConvCoef");
    ret = IDASetNonlinConvCoef(mIDAMemoryBlock, mIDANonlinConvCoef);
    if (check_retval(&ret, "IDASetNonlinConvCoef", 1)) throw CPS::Exception();
    mSLog->info("NonlinConvCoef set to {}", mIDANonlinConvCoef);

    // ### Main solver optional input functions ###
    /// specifies Maximum order for BDF method
    mSLog->info("Call IDASetMaxOrd");
    ret = IDASetMaxOrd(mIDAMemoryBlock, mIDAMaxBDFOrder);
    if (check_retval(&ret, "IDASetMaxOrd", 1)) throw CPS::Exception();
    mSLog->info("MaxOrd set to {}", mIDAMaxBDFOrder);

    /// specifies the maximum number of steps to be taken by the solver in its attempt to reach the next output
    mSLog->info("Call IDASetMaxNumSteps");
    ret = IDASetMaxNumSteps(mIDAMemoryBlock, mIDAMaxNumSteps);  //Max. number of timesteps until tout (-1 = unlimited)
    if (check_retval(&ret, "IDASetMaxNumSteps", 1)) throw CPS::Exception();
    mSLog->info("MaxNumSteps set to {}", mIDAMaxNumSteps);

    if (mVariableStepSize) {
        // specifies the minimum absolute value of the step size.
        mSLog->info("Call IDASetMinStep");
        ret = IDASetMinStep(mIDAMemoryBlock, mMinStepSize);
        if (check_retval(&ret, "IDASetMinStep", 1)) throw CPS::Exception();
        mSLog->info("Minimum absolute value of the step size set to {}", mMinStepSize);

        // specifies the maximum absolute value of the step size.
        mSLog->info("Call IDASetMaxStep");
        ret = IDASetMaxStep(mIDAMemoryBlock, mMaxStepSize);
        if (check_retval(&ret, "IDASetMaxStep", 1)) throw CPS::Exception();
        mSLog->info("Maximum absolute value of the step size set to {}", mMaxStepSize);
    } else {    // set IDASetMinStep == IDASetMaxStep == mTimeStep
        // specifies the minimum absolute value of the step size.
        mSLog->info("Call IDASetMinStep");
        ret = IDASetMinStep(mIDAMemoryBlock, mTimeStep);
        if (check_retval(&ret, "IDASetMinStep", 1)) throw CPS::Exception();
        mSLog->info("Minimum absolute value of the step size set to {}", mTimeStep);

        // specifies the maximum absolute value of the step size.
        mSLog->info("Call IDASetMaxStep");
        ret = IDASetMaxStep(mIDAMemoryBlock, mTimeStep);
        if (check_retval(&ret, "IDASetMaxStep", 1)) throw CPS::Exception();
        mSLog->info("Maximum absolute value of the step size set to {}", mTimeStep);
    }

    // specifies integration step size used on the first step == mTimeStep
    mSLog->info("Call IDASetInitStep");
    ret = IDASetInitStep(mIDAMemoryBlock, mTimeStep);
    if (check_retval(&ret, "IDASetInitStep", 1)) throw CPS::Exception();

    // specifies the maximum number of error test failures in attempting one step.
    mSLog->info("Call IDASetMaxErrTestFails");
    ret = IDASetMaxErrTestFails(mIDAMemoryBlock, mIDAMaxErrTestFails);
    if (check_retval(&ret, "IDASetMaxErrTestFails", 1)) throw CPS::Exception();

    // indicates whether or not to suppress algebraic variables in the local error test.
    /*
    In general, the use of this option (with suppressalg = SUNTRUE) is discouraged when solving DAE systems of index 1, 
    whereas it is generally encouraged for systems of index 2 or more.
    */
    mSLog->info("Call IDASetSuppressAlg");
    ret = IDASetSuppressAlg(mIDAMemoryBlock, mIDASetSuppressAlg);
    if (check_retval(&ret, "IDASetSuppressAlg", 1)) throw CPS::Exception();

    // specify algebraic/differential components in the state vector.
    mSLog->info("Call IDASetId");
    ret = IDASetId(mIDAMemoryBlock, mStateIDsVector);
    if (check_retval(&ret, "IDASetId", 1)) throw CPS::Exception();

    // If tstop is enabled,  then IDASolve returns the solution at tstop.
    // If itask is IDA_NORMAL, then the solver integrates from its current 
    // internal t value to a point at or beyond tout, then interpolates 
    // to t = tout and returns y(tret). In general, tret = tout.
    /*
    ret = IDASetStopTime(mIDAMemoryBlock, mTimestep);
    if (check_retval(&ret, "IDASetStopTime", 1)) {
        throw CPS::Exception();
	}
    */

    mSLog->info("Call IDA Solver Stuff");
    // Allocate and connect Matrix mJacobianMatrix and solver mLinearSolver to IDA
    mJacobianMatrix = SUNDenseMatrix(mNEQ, mNEQ, mSunctx);
    mLinearSolver = SUNLinSol_Dense(mStateVector, mJacobianMatrix, mSunctx);
    ret = IDASetLinearSolver(mIDAMemoryBlock, mLinearSolver, mJacobianMatrix);
    if (check_retval(&ret, "IDADlsSetLinearSolver", 1)) throw CPS::Exception();

    /*
    // Create sparse SUNMatrix for use in linear solves
    mJacobianMatrix = SUNSparseMatrix(mNEQ, mNEQ, mNEQ * mNEQ, CSC_MAT, mSunctx);
    if(check_retval((void *)mJacobianMatrix, "SUNSparseMatrix", 0)) throw CPS::Exception();

    // Create SuperLUMT SUNLinearSolver object (one thread)
    mLinearSolver = SUNLinSol_SuperLUMT(mStateVector, mJacobianMatrix, 1, mSunctx);
    if(check_retval((void *)mLinearSolver, "SUNLinSol_SuperLUMT", 0)) throw CPS::Exception();

    // Attach the matrix and linear solver
    retval = IDASetLinearSolver(memIDAMemoryBlockm, mLinearSolver, mJacobianMatrix);
    if (check_retval(&ret, "IDADlsSetLinearSolver", 1)) throw CPS::Exception();

    ** convert dense matrix to sparse matrix:
    https://sundials.readthedocs.io/en/latest/sunmatrix/SUNMatrix_links.html#c.SUNSparseFromBandMatrix
    */

    // Set the user-supplied Jacobian routine
    ret = IDASetJacFn(mIDAMemoryBlock, &DAESolver::jacobianFunctionWrapper);
    if(check_retval(&ret, "IDASetJacFn", 1)) throw CPS::Exception();

    /*
    // calculates corrected initial conditions
    ret = IDACalcIC(mIDAMemoryBlock, IDA_Y_INIT, mTimeStep);
    if (check_retval(&ret, "IDACalcIC", 1)) {
     	throw CPS::Exception();
	}
    */

    mSLog->info("--- Finished initialization --- \n");
    
    mSLog->info("--- Summary --- \n");
    //log name of state variables
    std::vector<std::string> stateVar_names(mNEQ);
    mSLog->info("\nVector of State Variables: ");
    int count_=0;
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
            mSLog->info("Voltage_{:s}", node->name());
            std::string nameVar = "Voltage_" + node->name();
            stateVar_names[count_] = nameVar;
            count_++;
        }
        else if (node->phaseType() == PhaseType::ABC) {
            mSLog->info("Voltage_{:s}-phase_A", node->name());
            std::string nameVar_a= "Voltage_" + node->name() + "-phase_A";
            stateVar_names[count_] = nameVar_a;
            count_++;
            mSLog->info("Voltage_{:s}-phase_B", node->name());
            std::string nameVar_b= "Voltage_" + node->name() + "-phase_B";
            stateVar_names[count_] = nameVar_b;
            count_++;
            mSLog->info("Voltage_{:s}-phase_C", node->name());
            std::string nameVar_c= "Voltage_" + node->name() + "-phase_C";
            stateVar_names[count_] = nameVar_c;
            count_++;
        }
    }
    for (auto comp: mDAEComponents) {
        for (int i=0; i<comp->getNumberOfStateVariables(); i++) {
            mSLog->info("Current_{:s}_stateVariable{}", 
                (std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp))->name(), i);
            std::string nameVar= "Current_" + std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp)->name() + "_stateVariable" + std::to_string(i);
            stateVar_names[count_] = nameVar;
            count_++;
        }
    }

    //Initial states of state variables
    mSLog->info("\nInitial values of state variables: ");
    for (int n=0; n<mNEQ; n++) {
        mSLog->info("Initial state of {:s}= {}", stateVar_names[n], sval[n]);
    }

    //Initial states of derivative of state variables
    mSLog->info("\nInitial values of derivative of state variables: ");
    for (int n=0; n<mNEQ; n++) {
        mSLog->info("Initial dstate of {:s}= {}", stateVar_names[n], s_dtval[n]);
    }

    // Log type of state variables
    mSLog->info("\nType of state variables: ");
    std::string typeVar = "";
    for (int n=0; n<mNEQ; n++) {
        if (state_var_types[n]==0.0)
            typeVar = "differential";
        else
            typeVar = "algebraic";
        mSLog->info("Type of {:s}: {:s}", stateVar_names[n], typeVar);
    }

    // Log absolute tolerances of state variables
    mSLog->info("\nAbsolute tolerances of state variables: ");
    for (int n=0; n<mNEQ; n++) {
        mSLog->info("Absolute tolerance of {:s}: {}", stateVar_names[n], absolute_tolerances[n]);
    }

    mSLog->flush();
}

template <typename VarType>
int DAESolver<VarType>::residualFunctionWrapper(realtype current_time, 
    N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data)
{
    DAESolver *self = reinterpret_cast<DAESolver *>(user_data);
    return self->residualFunction(current_time, state, dstate_dt, resid, user_data);
}

template <typename VarType>
int DAESolver<VarType>::jacobianFunctionWrapper(realtype current_time,  
    realtype cj, N_Vector state, N_Vector dstate_dt, N_Vector res_vec, SUNMatrix JJ, void *user_data,
    N_Vector tempv1, N_Vector tempv2, N_Vector tempv3)
{
    DAESolver *self = reinterpret_cast<DAESolver *>(user_data);
    return self->calculateJacobianMatrix(current_time, cj, state, dstate_dt, JJ);
}

template <typename VarType>
int DAESolver<VarType>::residualFunction(realtype current_time, 
    N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data)
{
    // Reset Offset of nodes
    mOffsets[0]=0;
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
            mOffsets[0] +=1;
        }
        else if (node->phaseType() == PhaseType::ABC) {
            mOffsets[0] +=3;
        }
    }
    mOffsets[1] = 0;    // Reset Offset of componentes

    //reset residual functions of nodes (nodal equations)
    realtype *residual = NULL;
    residual  = N_VGetArrayPointer(resid);
    for (int i=0; i<mOffsets[0]; i++) {
        residual[i] = 0;
    }

    // Call all registered component residual functions
    IDAGetCurrentStep(mIDAMemoryBlock, &mNextIntegrationStep);
    IDAGetCurrentTime(mIDAMemoryBlock, &mCurrentInternalTime);
    for (auto resFn : mResidualFunctions) {
        resFn(mCurrentInternalTime, NV_DATA_S(state), NV_DATA_S(dstate_dt), NV_DATA_S(resid), mOffsets);
    }

    mSLog->flush();
    // If successful; positive value if recoverable error, negative if fatal error
    // TODO: Error handling
    return 0;
}

template <typename VarType>
int DAESolver<VarType>::calculateJacobianMatrix(realtype current_time,  
    realtype cj, N_Vector state, N_Vector dstate_dt, SUNMatrix JJ) {

    // Reset Offset of nodes
    mOffsets[0]=0;
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
            mOffsets[0] +=1;
        }
        else if (node->phaseType() == PhaseType::ABC) {
            mOffsets[0] +=3;
        }
    }
    mOffsets[1] = 0;    // Reset Offset of componentes

    // Call all registered component residual functions
    for (auto jacobianFn : mJacobianFunctions)
        jacobianFn(current_time, NV_DATA_S(state), NV_DATA_S(dstate_dt), JJ, cj, mOffsets);

    // If successful; positive value if recoverable error, negative if fatal error
    // TODO: Error handling
    return 0;
}

template <typename VarType>
Real DAESolver<VarType>::step(Real time) {
    int ret=0;
    mSimTime = time;

    // perform 1 ida step
    ret = IDASolve(mIDAMemoryBlock, mSimTime, &mTimeReachedSolver,
        mStateVector, mDerivativeStateVector, IDA_NORMAL);
    
    //PrintOutput(mIDAMemoryBlock, mTimeReachedSolver, mStateVector);

    ///  Log sim statistics
    IDAGetCurrentTime(mIDAMemoryBlock, &mCurrentInternalTime);
    IDAGetNumSteps(mIDAMemoryBlock, &mNumberStepsIDA);
    IDAGetNumResEvals(mIDAMemoryBlock, &mNumberCallsResidualFunctions);
    IDAGetNumNonlinSolvIters(mIDAMemoryBlock, &mNonLinearIters);
    IDAGetLastStep(mIDAMemoryBlock, &mLastIntegrationStep);
    IDAGetCurrentStep(mIDAMemoryBlock, &mNextIntegrationStep);
    IDAGetLastOrder(mIDAMemoryBlock, &mOrderLastStep);
    IDAGetCurrentOrder(mIDAMemoryBlock, &mOrderNextStep);
    IDAGetErrWeights(mIDAMemoryBlock, mErrorWeights);
    IDAGetEstLocalErrors(mIDAMemoryBlock, mEstLocalErrors);
    IDAGetNumLinSolvSetups(mIDAMemoryBlock, &mNumLinSolvSetups);
    IDAGetNumErrTestFails(mIDAMemoryBlock, &mNumErrTestFails);
    IDAGetNumStepSolveFails(mIDAMemoryBlock, &mNumStepSolveFails);
    std::string estimatedAndWeightedErrors_str;
    printEstimatedAndWeightedErrors(estimatedAndWeightedErrors_str);

    mSLog->info(
            "\n###############Simulation statistics ####################"
            "\nCurrent internal time reached by the solver: {}"
            "\nCumulative number of internal steps: {}"
            "\nCumulative number of calls residual function: {}"
            "\nCumulative number of nonlinear iterations performed: {}"
            "\nCumulative number of calls made to the linear solver’s setup function (total so far): {}"
            "\nCumulative number of local error test failures that have occurred (total so far): {}"
            "\nNumber of failed steps due to a nonlinear solver failure: {}"
            "\nTime Reached Solver: {}"
            "\nOrder used during the last step: {}"
            "\nOrder to be attempted on the next step: {}"
            "\nIntegration step size taken on the last internal step: {}"
            "\nStep size to be attempted on the next step: {}"
            "{:s}",
            mCurrentInternalTime, 
            mNumberStepsIDA, 
            mNumberCallsResidualFunctions,
            mNonLinearIters,
            mNumLinSolvSetups,
            mNumErrTestFails,
            mNumStepSolveFails,
            mTimeReachedSolver,
            mOrderLastStep,
            mOrderNextStep,
            mLastIntegrationStep,
            mNextIntegrationStep,
            estimatedAndWeightedErrors_str
        );
    mSLog->flush(); 
    //IDAPrintAllStats(mIDAMemoryBlock);

    if (ret != IDA_SUCCESS) {
        std::string error_msg;
        get_error_str(ret, mIDAMaxNumSteps, mIDAMaxErrTestFails, mIDAMaxConvFails, error_msg);
        mSLog->info(
            "\nIDA Error: {}"   
            "\nCurrent internal time reached by the solver: {}"
            "\nSimulation finished!!",
            error_msg,
            mCurrentInternalTime
        );
        mSLog->flush();
        throw CPS::Exception();
    }
    
    updateVoltageAndCurrents();

    return mCurrentInternalTime;
}

template <typename VarType>
void DAESolver<VarType>::updateVoltageAndCurrents() {
     // update node voltages
    realtype *sval = N_VGetArrayPointer(mStateVector);
    realtype *dstate_val = N_VGetArrayPointer(mDerivativeStateVector);
    mOffsets[0] = 0;             // Reset Offset of nodes

    // update voltage of nodes
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
            node->setVoltage(sval[mOffsets[0]], PhaseType::Single);
            node->setdVoltage(dstate_val[mOffsets[0]]);
            mOffsets[0] +=1;
        }
        else if (node->phaseType() == PhaseType::ABC) {
            node->setVoltage(sval[mOffsets[0]], PhaseType::A);
            node->setdVoltage(dstate_val[mOffsets[0]], PhaseType::A);
            mOffsets[0] +=1;
            node->setVoltage(sval[mOffsets[0]], PhaseType::B);
            node->setdVoltage(dstate_val[mOffsets[0]], PhaseType::B);
            mOffsets[0] +=1;
            node->setVoltage(sval[mOffsets[0]], PhaseType::C);
            node->setdVoltage(dstate_val[mOffsets[0]], PhaseType::C);
            mOffsets[0] +=1;
        }
    }

    // update components
    mOffsets[1] = mOffsets[0];      // Reset Offset of componentes
    for (auto comp : mDAEComponents) { 
        comp->daePostStep(mSimTime, sval, dstate_val, mOffsets[1]);
    }
}

template <typename VarType>
Task::List DAESolver<VarType>::getTasks() {
    Task::List l;
    l.push_back(std::make_shared<DAESolver<VarType>::SolveStep>(*this));
    l.push_back(std::make_shared<DAESolver<VarType>::LogTask>(*this));
    return l;
}

template <typename VarType>
DAESolver<VarType>::~DAESolver() {
    // Releasing all memory allocated by IDA
    IDAFree(&mIDAMemoryBlock);
    N_VDestroy(mStateVector);
    N_VDestroy(mDerivativeStateVector);
    N_VDestroy(mStateIDsVector);
    N_VDestroy(mAbsoluteTolerances);
    N_VDestroy(mErrorWeights);
    N_VDestroy(mEstLocalErrors);
    SUNLinSolFree(mLinearSolver);
    SUNMatDestroy(mJacobianMatrix);
}

template <typename VarType>
void DAESolver<VarType>::printEstimatedAndWeightedErrors(std::string& ret) {
    realtype *errorsWeight = NULL;
    realtype *estLocalErrors = NULL;
    Matrix weightedErrors= Matrix::Zero(mNEQ, 1);
    errorsWeight  = N_VGetArrayPointer(mErrorWeights);
    estLocalErrors = N_VGetArrayPointer_Serial(mEstLocalErrors);
    for (int i=0; i<mNEQ; i++) 
        weightedErrors(i,0) = errorsWeight[i]*estLocalErrors[i];
        
    //get vector with names of the state variables
    std::string nameVariables[mNEQ];
    int counter = 0, counter2=0;
    for (auto node : mNodes) {
        if (node->phaseType() == PhaseType::Single) {
            nameVariables[counter++] = node->name();
        }
        else if (node->phaseType() == PhaseType::ABC) {
            nameVariables[counter++] = node->name() + "-phase_A";
            nameVariables[counter++] = node->name() + "-phase_B";
            nameVariables[counter++] = node->name() + "-phase_C";
        }
    }
    
    for (auto comp: mDAEComponents) {
        for (int i=0; i<comp->getNumberOfStateVariables(); i++) {
            nameVariables[counter++] =  mSystem.mComponents[counter2]->name() + "_stateVariable" + std::to_string(i);
        }
        counter2++;
    }

    ret = "\nEstimated local errors:\n";
    for (int i=0; i<mNEQ; i++) 
        ret += "\t" + nameVariables[i] + ": " + std::to_string(estLocalErrors[i]) + "\n";
    ret += "Error weights:\n";
    for (int i=0; i<mNEQ; i++) 
        ret += "\t" + nameVariables[i] + ": " + std::to_string(errorsWeight[i]) + "\n";
    ret += "Weighted error:\n";
    for (int i=0; i<mNEQ; i++)
        ret += "\t" + nameVariables[i] + ": " + std::to_string(weightedErrors(i,0)) + "\n";
}

static void get_error_str(int error_code, int IDAMaxNumSteps, int IDAMaxErrTestFails, int IDAMaxConvFails, std::string& ret) {
    
    switch(error_code) {
        case IDA_TOO_MUCH_WORK:
            ret = "The solver took mIDAMaxNumSteps (=" + std::to_string(IDAMaxNumSteps) + ") internal steps but could not reach tout";
            break;
        case IDA_TOO_MUCH_ACC:
            ret = "The solver could not satisfy the accuracy demanded by the user for some internal step";
            break;
        case IDA_ERR_FAIL:
            ret = "Error test failures occurred too many times (mIDAMaxErrTestFails = " + std::to_string(IDAMaxErrTestFails) + ") during one internal time step or occurred with integration_time = hmin";
            break;
        case IDA_CONV_FAIL:
            ret = "Convergence test failures occurred too many times (" + std::to_string(IDAMaxConvFails) + ") during one internal time step or occurred with integration_time = hmin";
            break;
        case IDA_LINIT_FAIL:
            ret = "The linear solver’s initialization function failed";
            break;
        case IDA_LSETUP_FAIL:
            ret = "The linear solver’s setup function failed in an unrecoverable manner";
            break;
        case IDA_LSOLVE_FAIL:
            ret = " The linear solver’s solve function failed in an unrecoverable manner";
            break;
        case IDA_RES_FAIL:
            ret = "The user’s residual function returned a nonrecoverable error flag";
            break;
        case IDA_REP_RES_ERR:
            ret = "The user’s residual function repeatedly returned a recoverable error flag, but the solver was unable to recove";
            break;
        case IDA_RTFUNC_FAIL:
            ret = " The rootfinding function failed";
            break;
        case IDA_CONSTR_FAIL:
            ret = "The inequality constraints were violated and the solver was unable to recover";
            break;
        case IDA_FIRST_RES_FAIL:
            ret = "The user’s residual function returned a recoverable error flag on the first call, but IDACalcIC() was unable to recover";
            break;
        case IDA_LINESEARCH_FAIL:
            ret = "The linesearch algorithm failed to find a solution with a step larger than steptol in weighted RMS norm, and within the allowed number of backtracks";
            break;
        case IDA_NO_RECOVERY:
            ret = " The user’s residual function, or the linear solver’s setup or solve function had a recoverable error, but IDACalcIC() was unable to recover";
            break;
        case IDA_MEM_NULL:
            ret = "The ida_mem argument was NULL";
            break;
        case IDA_MEM_FAIL:
            ret = "A memory allocation failed";
            break;
        case IDA_ILL_INPUT:
            ret = "The function g is NULL, but nrtfn > 0";
            break;
        case IDA_NO_MALLOC:
            ret = "Memory space for the IDA solver object was not allocated through a previous call to IDAInit()";
            break;
        default:
            ret = "Unidentified Error Occurred (" + std::to_string(error_code) + ")";
            break;
    }
}

static int check_retval(void *returnvalue, const char *funcname, int opt) {
    int *retval;
    /* Check if SUNDIALS function returned NULL pointer - no memory allocated */
    if (opt == 0 && returnvalue == NULL) {
        std::cout << "\nSUNDIALS_ERROR: " << funcname << "() failed - returned NULL pointer" << std::endl;
        return(1);
    } else if (opt == 1) {
        /* Check if retval < 0 */
        retval = (int *) returnvalue;
        if (*retval < 0) {
            std::cout << "\nSUNDIALS_ERROR: " << funcname << "() failed with retval = " << *retval << std::endl;
            return(1);
        }
    } else if (opt == 2 && returnvalue == NULL) {
        /* Check if function returned NULL pointer - no memory allocated */
         std::cout << "\nMEMORY_ERROR: " << funcname << "() failed - returned NULL pointer" << std::endl;
        return(1);
    }

    return (0);
}

/*
static void PrintOutput(void *mem, realtype t, N_Vector y)
{
  realtype *yval;
  int retval, kused;
  long int nst;
  realtype hused;

  yval  = N_VGetArrayPointer(y);

  retval = IDAGetLastOrder(mem, &kused);
  check_retval(&retval, "IDAGetLastOrder", 1);
  retval = IDAGetNumSteps(mem, &nst);
  check_retval(&retval, "IDAGetNumSteps", 1);
  retval = IDAGetLastStep(mem, &hused);
  check_retval(&retval, "IDAGetLastStep", 1);
#if defined(SUNDIALS_EXTENDED_PRECISION)
  printf("%10.4Le %12.4Le %12.4Le %12.4Le | %3ld  %1d %12.4Le\n",
         t, yval[0], yval[1], yval[2], nst, kused, hused);
#elif defined(SUNDIALS_DOUBLE_PRECISION)
  printf("%10.4e %12.4e %12.4e %12.4e | %3ld  %1d %12.4e\n",
         t, yval[0], yval[1], yval[2], nst, kused, hused);
#else
  printf("%10.4e %12.4e %12.4e %12.4e | %3ld  %1d %12.4e\n",
         t, yval[0], yval[1], yval[2], nst, kused, hused);
#endif
}
*/

template class DPsim::DAESolver<Real>;
template class DPsim::DAESolver<Complex>;
