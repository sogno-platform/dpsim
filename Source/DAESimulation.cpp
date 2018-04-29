
#include "DAESimulation.h"


Simulation::DAESimulation(String name, SystemTopology system, Real dt, Real tfinal) : DAESys(system), mName(name)
{

	//defines offset vector which is composed as follows:
	//offset[0]= # nodal voltage equations
	//offset[1]= # of componets and their respective equations (1 per component for now as inductance is not yet considered) 
	
	offsets.push_back(0);
	offsets.push_back(0);
}

void DAESimulation::initialize(Component::List newComponents)
{
	Int maxNode = 0;
	Int currentVirtualNode = 0;

	//mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;

	// Calculate the mNumber of nodes by going through the list of components
	// TODO we use the values from the first component vector right now and assume that
	// these values don't change on switches
	int maxNode = 0;
	for (auto comp : newComponents) {
		// determine maximum node in component list
		if (comp->getNode1() > maxNode) {
			maxNode = comp->getNode1();
		}
		if (comp->getNode2() > maxNode) {
			maxNode = comp->getNode2();
		}
	}

	if (mNodes.size() == 0) {
		// Create Nodes for all indices
		mNodes.resize(maxNode + 1, nullptr);
		for (int index = 0; index < mNodes.size(); index++)
			mNodes[index] = std::make_shared<Node>(index);

		for (auto comp : newComponents) {
			std::shared_ptr<Node> node1, node2;
			if (comp->getNode1() < 0)
				node1 = mGnd;
			else
				node1 = mNodes[comp->getNode1()];
			if (comp->getNode2() < 0)
				node2 = mGnd;
			else
				node2 = mNodes[comp->getNode2()];

			comp->setNodes(Node::List{ node1, node2 });
		}
	}

	// TO-DO: Initialize right side vector and components
}

 
void DAESimulation::run()
{
	void *mem = NULL;
	mem = IDACreate();
	int retval1 = IDAInit(mem, residualFunction, t0, state, dstate_dt); //TO-DO: get remaining parameters
	// Do error checking with retval 1
	/*
		Add IDA tolerances 
	*/
	int NEQ = mComponents.size(); // is this implemented?
	retval = IDADense(mem, NEQ); // choose right solver
	int iout = 0;
	int tout = 1; //final time
	while(1){

		int retval2 = IDASolve(mem, tout, &tret, state, dstate_dt, IDA_NORMAL); //TO-DO: implement parameters 
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
	for (auto node : mNodes){ 
		double residual[]=NVECTOR_DATA(resid);
		double tempstate[]=NVECTOR_DATA(state);
		residual[offsets[0]]=tempstate[offsets[0]]-node->voltage;
		offsets[0]=offsets[0]+1;
	}
	for (auto comp : mComponents){  	//currently only supports DP_Resistor and DP_VoltageSource
		comp->residual(ttime, NVECTOR_DATA(state), NVECTOR_DATA(dstate_dt), NVECTOR_DATA(resid), offsets);
	}
		int ret=0;
	/*
	Do Error checking with variable ret
	*/
	return ret; //if successful; positive value if recoverable error, negative if fatal error
}