
#include "DAESolver.h"


DAESolver::DAESolver(String name,  SystemTopology system, Real dt) : DAESys(system), timestep(dt)
{

	// defines offset vector which is composed as follows:
	// offset[0]= # nodal voltage equations
	// offset[1]= # of componets and their respective equations (1 per component for now as inductance is not yet considered) 
	
	offsets.push_back(0);
	offsets.push_back(0);
}

void DAESolver::initialize(Component::List newComponents)
{
	for (auto comp : newComponents) {
			
		comp->initializePowerflow(DAESys.mSystemFrequency);
	}

	// TODO: move init functions from run
}

 
void DAESolver::run()
{

	 /* 
	 	All required variables : 
	    state = Vector of relevant variables for solving
		dstate_dt = derivates with respect to time of the state vector
	 	t0 = starting time (usually 0)
	 	tout = time for desired solution;
		rtol = relative tolerance
	 	tret = time IDA reached while solving
		abstol = scalar absolute tolerance 
		A = template Jacobian required for solver
		LS = linear solver object
	 */

	//Allocation of all variables required by IDA
	void *mem = NULL;
	N_Vector state, dstate_dt; //avtol ;
	state=state_dt = NULL;
	realtype t0, tout, rtol ,tret , *sval, *s_dtval, abstol //*atval;
	sval = s_dtval =NULL;
	SUNMatrix A;
	SUNLinearSolver LS;
	A = NULL;
	LS = NULL;
	int NEQ = DAESys.mComponents.size()+DAESys.mNodes.size(); 
	
	// allocate state vectors
	state = N_VNew_Serial(NEQ);
   
    dstate_dt = N_VNew_Serial(NEQ);

	avtol = N_VNew_Serial(NEQ);

	// set inital values of components
	initialize(DAESys.mComponents);


   /* Seting intial values for all required vectors. 
	The state vector is  defined as follows: 
	
	state[0]=node0_voltage
	state[1]=node1_voltage
	...
	state[n]=noden_voltage
	state[n+1]=component0_voltage
	state[n+2]=component0_inductance (not yet implemented)
	...
	state[m-1]=componentm_voltage
	state[m]=componentm_inductance (not yet implemented)
	
	*/

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

	/*
	atval = N_VGetArrayPointer_Serial(avtol);
		set inital values for absolute tolerance if noise differs for each component
	*/
	
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
	
	t0 = 0;
	tout=timestep;
	
	while(1){

		retval = IDASolve(mem, tout, &tret, state, dstate_dt, IDA_NORMAL);  
		
		if (retval == IDA_SUCCESS){ 
			tout += timestep;
		}
		else {
			std::cout <<"Ida Error"<<std::endl;
			break;
		}
		
		if(tout>=mFinalTime) break;
	}
	
	std::cout<<"Future Solution Vector"<<endl;
	//Freeing allocated memory
	 IDAFree(&mem);
	 //N_VDestroy(avtol);
	 N_VDestroy(state);
	 N_VDestroy(state_dt);
	 SUNLinSolFree(LS);
	 SUNMatDestroy(A);
	 
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

Real DAESolver::step(Real time, bool blocking){
	
	retval = IDASolve(mem, time, &tret, state, dstate_dt, IDA_NORMAL);  
		
	if (retval == IDA_SUCCESS){ 
		return time + mTimestep	
	}

	else {
		std::cout <<"Ida Error"<<std::endl;
		break;
	}
}