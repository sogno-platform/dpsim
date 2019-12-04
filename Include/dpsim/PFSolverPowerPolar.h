/** PFSolverPowerPolar
 * @author Jan Dinkelbach <jdinkelbach@eonerc.rwth-aachen.de>
 * @copyright 2019, Institute for Automation of Complex Power Systems, EONERC
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

#pragma once

#include <dpsim/PFSolver.h>

namespace DPsim {
    /// Powerflow solver class considering power mismatch and voltages in polar coordinates.
    class PFSolverPowerPolar : public PFSolver {
    protected:
        /// Solution vector of active power
        CPS::Vector sol_P;
        /// Solution vector of reactive power
		CPS::Vector sol_Q;
        /// Solution vector of voltage magnitude
		CPS::Vector sol_V;
        /// Solution vector of voltage angle
		CPS::Vector sol_D;
        /// Solution vector of representing sol_V and sol_D as complex quantity
		CPS::VectorComp sol_V_complex;
        /// Solution vector of representing sol_P and sol_Q as complex quantity
        CPS::VectorComp sol_S_complex;

        CPS::Vector Pesp;
        CPS::Vector Qesp;

        // Core methods
        /// Generate initial solution for current time step
        void generateInitialSolution(Real time, bool keep_last_solution = false);
        /// Calculate the Jacobian
        void calculateJacobian();
        /// Update solution in each iteration
        void updateSolution();
        /// Set final solution
        void setSolution();
        /// Calculate mismatch
        void calculateMismatch();
        
        // Helper methods
        /// Resize solution vector
        void resize_sol(CPS::Int n);
        /// Resize complex solution vector
		void resize_complex_sol(CPS::Int n);
        /// Calculate real part of voltage from sol_V and sol_D
		CPS::Real sol_Vr(CPS::UInt k);
        /// Calculate imaginary part of voltage from sol_V and sol_D
        CPS::Real sol_Vi(CPS::UInt k);
        /// Calculate complex voltage from sol_V and sol_D
		CPS::Complex sol_Vcx(CPS::UInt k);
        /// Calculate active power at a bus from current solution
        CPS::Real P(CPS::UInt k);
        /// Calculate the reactive power at a bus from current solution
        CPS::Real Q(CPS::UInt k);
        /// Calculate P and Q at slack bus from current solution
        void calculatePAndQAtSlackBus();
        /// Calculate the reactive power at all PV buses from current solution
        void calculateQAtPVBuses();
        /// Calculate branch flows from current solution and store them in line and transformer components
        void calculateBranchFlow();
        /// Calculate nodal power injections and store them in first line or transformer (in case no line is connected), so that lower level classes (Node, TopologicalTerminal) can stay untouched
        void calculateNodalInjection();
    public:
        /// Constructor to be used in simulation examples.
        PFSolverPowerPolar(CPS::String name, CPS::SystemTopology system, CPS::Real timeStep, CPS::Logger::Level logLevel);
        ///
		virtual ~PFSolverPowerPolar() { };
    };
}