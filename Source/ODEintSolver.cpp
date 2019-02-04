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

#include <dpsim/ODEintSolver.h>


using namespace DPsim;

ODEintSolver::ODEintSolver(String name, CPS::ODEintInterface::Ptr comp, Real dt, Real t0) :
        mComponent(comp), mTimestep(dt){
        times.push_back(t0);
        self = this; // sets static pointer to current object
        ProbDim = comp->num_states();

        curSolution.resize(ProbDim);

        //register all system functions
        system.push_back(
                [comp](const double y[],  double ydot[],  const double t){
                    comp->odeint(y, ydot, t);
                });

}



Real ODEintSolver::step(Real time) {

    mComponent->pre_step(); ///write inital values into the mState Vector
    curSolution.assign(mComponent->state_vector(), mComponent->state_vector()+ProbDim);
    Real NextTime = time + mTimestep;

    std::cout<<"Current Time "<<NextTime<<std::endl;

    stepper.do_step(&ODEintSolver::StateSpaceWrapper,curSolution, time, mTimestep); ///solve ODE for time + mTimestep TODO: implement sysfunc
    mComponent->set_state_vector(curSolution);///Writes the current solution back into the component
    solution.push_back(curSolution); /// Logs current solution
    times.push_back(time); ///Logs current time

    // TODO: add proper logging
    mComponent->post_step();
    return NextTime;
}

void ODEintSolver::StateSpaceWrapper( const std::vector<double> y, std::vector<double> ydot, double t){

        for(auto comp : self->system){ // call system functions of the components with the state vector
                comp(y.data(), ydot.data(), t);
        }
}

ODEintSolver::~ODEintSolver() {
}

