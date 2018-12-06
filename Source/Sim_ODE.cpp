/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

 #include <dpsim/MNASolver.h>

 #ifdef WITH_CIM
   #include <cps/CIM/Reader.h>
 #endif

 #ifdef WITH_SUNDIALS
   #include <dpsim/DAESolver.h>
 #endif

 #include <dpsim/Sim_ODE.h>

 //using namespace CPS;
 using namespace DPsim;

 Sim_ODE::Sim_ODE(String name,
   CPS::SystemTopology system,
   Real timeStep, Real finalTime,
   std::vector<std::shared_ptr<ODESolver> > ODESolverList,
   Domain domain, Solver::Type solverType,
   Logger::Level logLevel) :
   Simulation(name, system, timeStep, finalTime, domain, solverType, logLevel),
   mODESolverList(ODESolverList){}

Real Sim_ODE::step(){
  Real nextTime; //Auxiliary variable necessary?

 #ifdef WITH_SHMEM
 	for (auto ifm : mInterfaces) {
 		if (mTimeStepCount % ifm.downsampling == 0)
 			ifm.interface->readValues(ifm.sync);
 	}
 #endif

 	mEvents.handleEvents(mTime);
  //for testing purposes
  /*for(auto ode_solver:mODESolverList){
    std::cout <<ode_solver->get_comp()->get_data() << std::endl;
  }*/

  // ODE-Solver:
  for (auto ode_solver:mODESolverList){
    nextTime = ode_solver->step(mTime); //possibly some pre-coperation necessary?
    ode_solver->log(mTime); //currently not supported
  }

  //DAE-Solver:
  nextTime = mSolver->step(mTime);
	mSolver->log(mTime);

  #ifdef WITH_SHMEM
  	for (auto ifm : mInterfaces) {
  		if (mTimeStepCount % ifm.downsampling == 0)
  			ifm.interface->writeValues();
  	}
  #endif

  	for (auto lg : mLoggers) {
  		if (mTimeStepCount % lg.downsampling == 0) {
  			lg.logger->log(mTime);
          }
  	}

  	mTime = nextTime;
  	mTimeStepCount++;

    return mTime;
}
