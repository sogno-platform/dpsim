/** SynGenDPBalancedResLoad Example
 *
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

#include <DPsim.h>
#include<dpsim/ODEintSolver.h>

using namespace DPsim;
using namespace CPS::DP;
using namespace CPS::DP::Ph3;

/*###### For execution at least one command line argument is required: ###########
	First argument: binary flag (0 or 1),
	0: The Generator Equations are solved without using the ODE-Simulation Class
	1: The Generator Equations are solved using the ODE-Simulation Class

    This is a general Test of the ODEintSolver Class. TODO: Add ODEint Solver option to Sim_ODE interface
	*/



int main(int argc, char* argv[]) {
    // Define simulation parameters
    Real om = 2.0 * PI * 60.0;
    Real timeStep = 0.00005; //initial: 0.00005
    Real finalTime = 0.1;
    Real curTime = timeStep;
    String simName = "DP_SynchronGenerator_dq_ThreePhFault";
    // Define machine parameters in per unit
    Real nomPower = 555e6;
    Real nomPhPhVoltRMS = 24e3;
    Real nomFreq = 60;
    Real nomFieldCurr = 1300;
    Int poleNum = 2;
    Real H = 3.7;
    Real Rs = 0.003;
    Real Ll = 0.15;
    Real Lmd = 1.6599;
    Real Lmq = 1.61;
    Real Rfd = 0.0006;
    Real Llfd = 0.1648;
    Real Rkd = 0.0284;
    Real Llkd = 0.1713;
    Real Rkq1 = 0.0062;
    Real Llkq1 = 0.7252;
    Real Rkq2 = 0.0237;
    Real Llkq2 = 0.125;
    // Initialization parameters
    Real initActivePower = 300e6;
    Real initReactivePower = 0;
    Real initTerminalVolt = 24000 / sqrt(3) * sqrt(2);
    Real initVoltAngle = -PI / 2;
    Real fieldVoltage = 7.0821;
    Real mechPower = 300e6;


    /// Use seperate ODE-Solver and Simulation Class->true ; use directly built-in solver: false
    bool ode_class;
    /// Pass on command line '1' for execution with ODE-Class; else use built-in solver
    assert(argc>=2);
    if(atoi(argv[1])==1){
        ode_class=true; // simulate with ode-class
    }else{
        ode_class=false; // simulate without ode-class (use built in solver)
    }

    // Components
    auto gen = Ph3::SynchronGeneratorDQ::make("DP_SynGen_dq_ThreePhFault_SynGen", ode_class);
    gen->setParametersFundamentalPerUnit(nomPower, nomPhPhVoltRMS, nomFreq, poleNum, nomFieldCurr,
                                    Rs, Ll, Lmd, Lmq, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, H,
                                    initActivePower, initReactivePower, initTerminalVolt, initVoltAngle, fieldVoltage, mechPower);

    ODEintSolver sim(simName, gen, timeStep, timeStep);

    while (curTime < finalTime){

        curTime = sim.step(curTime);
        for (auto sol : sim.curSolution)
            std::cout<< sol<<std::endl;

    }

    return 0;
}
