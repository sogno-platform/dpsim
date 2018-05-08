
#include "DAESimulation.h"


DAESimulation::DAESimulation(String name, SystemTopology system, Real dt, Real tfinal) : DAESys(system), mName(name)
{

	//defines offset vector which is composed as follows:
	//offset[0]= # nodal voltage equations
	//offset[1]= # of componets and their respective equations (1 per component for now as inductance is not yet considered) 
	
	offsets.push_back(0);
	offsets.push_back(0);
	mFinalTime=tfinal;
}

void DAESimulation::initialize(Component::List newComponents)
{
	for (auto comp : newComponents) {
			
		comp->initializePowerflow(DAESys.mSystemFrequency);
	}

	// TO-DO: Get more Ida initialization stuff from run function
}

 
void DAESimulation::run()
{
	void *mem = NULL;
	N_Vector state, state_dt, avtol ;
	state=state_dt = avtol=NULL;
	realtype tout, rtol, *sval, *s_dtval, *atval;
	sval = s_dtval=  atval =NULL;
	int NEQ = DAESys.mComponents.size(); 
	
	state = N_VNew_Serial(NEQ);
    if(check_flag((void *)state, "N_VNew_Serial", 0)) return;
    state_dt = N_VNew_Serial(NEQ);
    if(check_flag((void *)state_dt, "N_VNew_Serial", 0)) return;
	avtol = N_VNew_Serial(NEQ);
    if(check_flag((void *)avtol, "N_VNew_Serial", 0)) return;

	///set intial values for state vector 
	initialize(DAESys.mComponents);

	/*vector definintion: 
	
	state[0]=node0_voltage
	state[1]=node1_voltage
	....
	state[n]=noden_voltage
	state[n+1]=component0_voltage
	state[n+2]=component0_inductance (not yet implemented)
	...
	state[m-1]=componentm_voltage
	state[m]=componentm_inductance
	
	*/
	int counter = 0;
	sval = N_VGetArrayPointer_Serial(state);
	for (auto node : DAESys.mNodes) {

		sval[counter++]=node->getVoltage();

	}
	
	for (auto comp : DAESys.mComponents){

		sval[counter++]=comp->getVoltage();
		//sval[counter++]=component inductance;

	}	
	
	s_dtval = N_VGetArrayPointer_Serial(state_dt);
	/*
		set inital values for state derivative
		for now all equal to 0
	*/
	for (int i =0, i<(DAESys.mNodes.size()+DAESys.mComponents.size()-1), i++)
		s_dtval[i] = 0; //TODO: add derivative calculation


	atval = N_VGetArrayPointer_Serial(avtol);
	/*

		set inital values for absolute tolerance
	*/

	 rtol = RCONST(1.0e-4); //set relative tolerance, is this correct?

	int t0 = 0;
	tout1 = mFinalTime;
	mem = IDACreate();

	int retval1 = IDAInit(mem, DAE_residualFunction, t0, state, dstate_dt); //TO-DO: get remaining parameters
	// Do error checking with retval 1

 	N_VDestroy_Serial(avtol);

	retval = IDADense(mem, NEQ); // choose right solver
	
	int iout = 0;
	int tout = 1; //final time
	while(1){

		int retval2 = IDASolve(mem, tout, &tret, state, dstate_dt, IDA_NORMAL);  
		if(retval2==1) return 1;
		if (retval == IDA_SUCCESS){ 
			iout++;
			tout+= 0.1; //10 iterations here
		}
		if (iout == maxIt) break; //maxIt = max number of iterations
	}
	std::cout<<"Future Solution Vector"<<endl;
	 IDAFree(&mem);
	 /*
		Free state and dstate_dt
	 */
}

int DAESimulation::DAE_residualFunction(realtype ttime, N_Vector state, N_Vector dstate_dt, N_Vector resid, void *user_data)
{
	for (auto node : DAESys.mNodes){ 
		double residual[]=NVECTOR_DATA(resid);
		double tempstate[]=NVECTOR_DATA(state);
		residual[offsets[0]]=tempstate[offsets[0]]-node->voltage;
		offsets[0]=offsets[0]+1;
	}
	for (auto comp : DAESys.mComponents){  	//currently only supports DP_Resistor and DP_VoltageSource
		comp->residual(ttime, NVECTOR_DATA(state), NVECTOR_DATA(dstate_dt), NVECTOR_DATA(resid), offsets);
	}
		int ret=0;
	/*
	Do Error checking with variable ret
	*/
	return ret; //if successful; positive value if recoverable error, negative if fatal error
}