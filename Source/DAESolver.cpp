
#include "DAESolver.h"


DAESolver::DAESolver(String name,  SystemTopology system, Real dt, Real t0) :  mTimestep(dt)
{

     DAESolver::DAESys = system;
	// defines offset vector of the residual which is composed as follows:
	// offset[0] = # nodal voltage equations
	// offset[1] = # of componets and their respective equations (1 per component for now as inductance is not yet considered) 
	offsets.push_back(0);
    offsets.push_back(0);
    NEQ = DAESys.mComponents.size() + (2 * DAESys.mNodes.size());
	 
	// set inital values of all required variables and create IDA solver environment
    for(auto comp : DAESys.mComponents){
       PowerComponent<Real>::Ptr ncomp = std::dynamic_pointer_cast< PowerComponent<Real> >(comp);
       mComponents.push_back(ncomp);
    }
    initialize(t0);
}

void DAESolver::initialize(Real t0)
{
	//set inital values of all components
    for (auto comp : mComponents) {
			
		comp->initializePowerflow(DAESys.mSystemFrequency);
	}

	// allocate state vectors
	state = N_VNew_Serial(NEQ);
   
    dstate_dt = N_VNew_Serial(NEQ);

	//avtol = N_VNew_Serial(NEQ);
	

	realtype *sval = NULL, *s_dtval=NULL ; 
	
	int counter = 0;

	sval = N_VGetArrayPointer_Serial(state); 
	for (auto node : DAESys.mNodes) {
        //initialize nodal values of state vector
        sval[counter++] = node->getVoltage();
	}

    for (auto comp : mComponents){
		//initialize  component values of state vector
        sval[counter++] = comp->getVoltage();
		//sval[counter++]=component inductance;
	}	
	
    for (int j = 1; j<NEQ; j++){
        //initialize nodal current equations
        sval[counter++] = 0 ;
    }

	s_dtval = N_VGetArrayPointer_Serial(dstate_dt); 

	/*
		set inital values for state derivative
		for now all equal to 0
	*/

    for (int i =0; i<(NEQ-1); i++)
		s_dtval[i] = 0; // TODO: add derivative calculation

	rtol = RCONST(1.0e-6); // set relative tolerance
	abstol = RCONST(1.0e-1); // set absolute error

	mem = IDACreate();

    int retval = IDAInit(mem, &DAESolver::DAE_residualFunction, t0, state, dstate_dt);
	// if(check_flag(&retval, "IDAInit", 1)) return(1);

	retval = IDASStolerances(mem, rtol, abstol);
 	// if(check_flag(&retval, "IDASStolerances", 1)) return(1);


	//allocate and connect Matrix A and solver LS to IDA 
	A = SUNDenseMatrix(NEQ, NEQ); 
	LS = SUNDenseLinearSolver(state, A);
	retval = IDADlsSetLinearSolver(mem, LS, A);
	
}


int DAESolver::DAE_residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data)
{
	offsets[0]=0;  //reset Offset
    offsets[1]=0;  //reset Offset
	for (auto node : DAESys.mNodes){ //solve for all node Voltages
        double * residual = NV_DATA_S(resid);
        double * tempstate = NV_DATA_S(state);
        residual[offsets[0]] = tempstate[offsets[0]]-node->voltage;
        offsets[0] += 1;
	}
	
    for (auto comp : DAESys.mComponents){  	// currently only supports DP_Resistor and DP_VoltageSource
        comp->daeResidual(ttime, NV_DATA_S(state), NV_DATA_S(dstate_dt), NV_DATA_S(resid), offsets);
	}
		int ret=0;
	/*
	Do Error checking with variable ret
	*/

	return ret; // if successful; positive value if recoverable error, negative if fatal error
}

Real DAESolver::step(Real time){
	
    int retval = IDASolve(mem, time, &tret, state, dstate_dt, IDA_NORMAL);  //TODO: find alternative to IDA_NORMAL
		
	if (retval == IDA_SUCCESS){ 
		return time + mTimestep	;
	}

	else {
		std::cout <<"Ida Error"<<std::endl;
        return time + mTimestep	;
	}
}

DAESolver::~DAESolver(){
	//releasing all memory allocated by IDA
	 IDAFree(&mem);
	 N_VDestroy(state);
     N_VDestroy(dstate_dt);
	 SUNLinSolFree(LS);
	 SUNMatDestroy(A);
}
