
#include "DAESolver.h"


DAESolver::DAESolver(String name,  SystemTopology system, Real dt, Real t0) : DAESys(system), timestep(dt)
{

	// defines offset vector which is composed as follows:
	// offset[0]= # nodal voltage equations
	// offset[1]= # of componets and their respective equations (1 per component for now as inductance is not yet considered) 
	
	offsets.push_back(0);
	offsets.push_back(0);

	NEQ = DAESys.mComponents.size()+DAESys.mNodes.size();
	 
	// set inital values of all required variables and create IDA solver environment
	initialize(DAESys.mComponents,t0);
}

void DAESolver::initialize(Component::List newComponents, Real t0)
{
	//set inital values of all components
	for (auto comp : newComponents) {
			
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
		sval[counter++]=node->getVoltage();

	}
	
	for (auto comp : DAESys.mComponents){
		//initialize  component values of state vector
		sval[counter++]=comp->getVoltage();
		//sval[counter++]=component inductance;

	}	
	
	s_dtval = N_VGetArrayPointer_Serial(dstate_dt); 

	/*
		set inital values for state derivative
		for now all equal to 0
	*/

	for (int i =0, i<(DAESys.mNodes.size()+DAESys.mComponents.size()-1), i++)
		s_dtval[i] = 0; // TODO: add derivative calculation

	rtol = RCONST(1.0e-6); // set relative tolerance
	abstol = RCONST(1.0e-1); // set absolute error

	mem = IDACreate();

	int retval = IDAInit(mem, DAE_residualFunction, t0, state, dstate_dt); 
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
	for (auto node : DAESys.mNodes){ 
		double residual[]=NVECTOR_DATA(resid);
		double tempstate[]=NVECTOR_DATA(state);
		residual[offsets[0]]=tempstate[offsets[0]]-node->voltage;
		offsets[0]=offsets[0]+1;
	}

	for (auto comp : DAESys.mComponents){  	// currently only supports DP_Resistor and DP_VoltageSource
		comp->residual(ttime, NVECTOR_DATA(state), NVECTOR_DATA(dstate_dt), NVECTOR_DATA(resid), offsets);
	}
		int ret=0;
	/*
	Do Error checking with variable ret
	*/

	return ret; // if successful; positive value if recoverable error, negative if fatal error
}

Real DAESolver::step(Real time){
	
	retval = IDASolve(mem, time, &tret, state, dstate_dt, IDA_NORMAL);  //TODO: find alternative to IDA_NORMAL 
		
	if (retval == IDA_SUCCESS){ 
		return time + mTimestep	;
	}

	else {
		std::cout <<"Ida Error"<<std::endl;
		break;
	}
}

DAESolver::~DAESolver(){
	//releasing all memory allocated by IDA
	 IDAFree(&mem);
	 N_VDestroy(state);
	 N_VDestroy(state_dt);
	 SUNLinSolFree(LS);
	 SUNMatDestroy(A);
}