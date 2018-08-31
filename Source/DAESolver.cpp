
/*********************************************************************************
* @file
* @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
* @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
*
* CPowerSystems
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

    for (auto baseNode : DAESys.mNodes) {
        // Add nodes to the list and ignore ground nodes.
        if (!baseNode->isGround()) {
            auto node = std::dynamic_pointer_cast<Node<Real> >(baseNode);
            mNodes.push_back(node);
        }
    }

    UInt simNodeIdx = -1;
    for (UInt idx = 0; idx < mNodes.size(); idx++) {
        mNodes[idx]->simNodes()[0] = ++simNodeIdx;
        if (mNodes[idx]->getPhaseType() == PhaseType::ABC) {
            mNodes[idx]->simNodes()[1] = ++simNodeIdx;
            mNodes[idx]->simNodes()[2] = ++simNodeIdx;
        }
    }

    initialize(t0);
}

void DAESolver::initialize(Real t0)
{
	//set inital values of all components
    for (auto comp : mComponents) {
			
		comp->initializeFromPowerflow(DAESys.mSystemFrequency);
	}

	// allocate state vectors
	state = N_VNew_Serial(NEQ);
   
    dstate_dt = N_VNew_Serial(NEQ);

	//avtol = N_VNew_Serial(NEQ);
	

	realtype *sval = NULL, *s_dtval=NULL ; 
	
	int counter = 0;

	sval = N_VGetArrayPointer_Serial(state); 

    for (auto node : mNodes) {
        //initialize nodal values of state vector
        Real tempVolt = 0;
        tempVolt += std::real(node->initialVoltage()(0,0));
        sval[counter++] = tempVolt;

//        if (node->getPhaseType() == PhaseType::ABC) {
//           tempVolt += std::real(node->initialVoltage()(1,0));
//           tempVolt += std::real(node->initialVoltage()(2,0));
//
//        }
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

    if (retval == 0)std::cout<<"test"<<std::endl; //TODO: Remove this(only for compile purpose)
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

    for (auto node : mNodes){ //solve for all node Voltages
        double * residual = NV_DATA_S(resid);
        double * tempstate = NV_DATA_S(state);
        Real tempVolt = 0;
        tempVolt += std::real(node->getVoltage()(0,0));

//        if (node->getPhaseType() == PhaseType::ABC) {
//           tempVolt += std::real(node->getVoltage()(1,0));
//           tempVolt += std::real(node->getVoltage()(2,0));
//           sval[counter++] = tempVolt;
//        }

        residual[offsets[0]] = tempstate[offsets[0]]-tempVolt;
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
