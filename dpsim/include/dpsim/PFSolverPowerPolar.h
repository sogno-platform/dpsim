/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
        /// Calculate the SG's active and reactive power from current solution
        void setSGPower();
        /// Calculate branch flows from current solution and store them in line and transformer components
        void calculateBranchFlow();
        /// Calculate nodal power injections and store them in first line or transformer (in case no line is connected), so that lower level classes (Node, TopologicalTerminal) can stay untouched
        void calculateNodalInjection();
    public:
        /// Constructor to be used in simulation examples.
        PFSolverPowerPolar(CPS::String name, const CPS::SystemTopology &system, CPS::Real timeStep, CPS::Logger::Level logLevel);
        ///
		virtual ~PFSolverPowerPolar() { };
    };
}
