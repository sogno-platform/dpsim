/** PFSolver
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

#include <cmath>
#include <iterator>

#include <dpsim/Solver.h>
#include <dpsim/Scheduler.h>
#include "cps/SystemTopology.h"
#include "cps/Components.h"
#include "cps/Component.h"

namespace DPsim {
    /// Solver class using the nonlinear powerflow (PF) formulation.
    class PFSolver: public Solver {
    protected:
        /// System time step is constant for PF solver
        Real mTimeStep;
        /// Number of PQ nodes
        UInt mNumPQBuses = 0;
        /// Number of PV nodes
        UInt mNumPVBuses = 0;
        /// Number of PV nodes
        UInt mNumVDBuses = 0;
        /// Number of unknowns, defining system dimension
        UInt mNumUnknowns = 0;
        /// Vector of nodes characterized as PQ buses
        CPS::TopologicalNode::List mPQBuses;
        /// Vector of nodes characterized as PV buses
        CPS::TopologicalNode::List mPVBuses;
        /// Vector of nodes characterized as VD buses
        CPS::TopologicalNode::List mVDBuses;
        /// Vector with indices of PQ buses
        std::vector<CPS::UInt> mPQBusIndices;
        /// Vector with indices of PV buses
        std::vector<CPS::UInt> mPVBusIndices;
        /// Vector with indices of VD buses
        std::vector<CPS::UInt> mVDBusIndices;
        /// Vector with indices of both PQ and PV buses
        std::vector<CPS::UInt> mPQPVBusIndices;

        /// Admittance matrix
        CPS::SparseMatrixCompRow mY;

        /// Jacobian matrix 
        CPS::Matrix mJ;
        /// Solution vector
        CPS::Vector mX;
	    /// Vector of mismatch values
        CPS::Vector mF;

        /// System list
        CPS::SystemTopology mSystem;
        /// Vector of transformer components
        std::vector<std::shared_ptr<CPS::SP::Ph1::Transformer>> mTransformers;
        /// Vector of synchronous generator components
        std::vector<std::shared_ptr<CPS::SP::Ph1::SynchronGenerator>> mSynchronGenerators;
        /// Vector of load components
        std::vector<std::shared_ptr<CPS::SP::Ph1::Load>> mLoads;
        /// Vector of line components
        std::vector<std::shared_ptr<CPS::SP::Ph1::PiLine>> mLines;
        /// Vector of shunt components
        std::vector<std::shared_ptr<CPS::SP::Ph1::Shunt>> mShunts;
        /// Vector of external grid components
        std::vector<std::shared_ptr<CPS::SP::Ph1::externalGridInjection>> mExternalGrids;

        /// Solver tolerance
		Real mTolerance = 1e-8;
        /// Maximum number of iterations
		CPS::UInt mMaxIterations = 9;
        /// Actual number of iterations
		CPS::UInt mIterations;
        /// Base power of per-unit system
		CPS::Real mBaseApparentPower;
        /// Convergence flag
        CPS::Bool isConverged = false;
        /// Flag whether solution vectors are initialized 
        CPS::Bool solutionInitialized = false;
        /// Flag whether complex solution vectors are initialized
        CPS::Bool solutionComplexInitialized = false;

        /// Generate initial solution for current time step
        virtual void generateInitialSolution(Real time, bool keep_last_solution = false) = 0;
        /// Calculate mismatch
        virtual void calculateMismatch() = 0;
        /// Calculate the Jacobian
        virtual void calculateJacobian() = 0;
        /// Update solution in each iteration
        virtual void updateSolution() = 0;
        /// Set final solution
        virtual void setSolution() = 0;
        
        /// Initialization of the solver
        void initialize();
        /// Initialization of individual components
        void initializeComponents();
        /// Set apparent base power of per-unit system
        void setBaseApparentPower();
        /// Determine bus type for all buses
        void determinePFBusType();
        /// Compose admittance matrix
		void composeAdmittanceMatrix();
        /// Gets the real part of admittance matrix element
        CPS::Real G(int i, int j);
        /// Gets the imaginary part of admittance matrix element
        CPS::Real B(int i, int j);
        /// Solves the powerflow problem
        Bool solvePowerflow();
        /// Check whether below tolerance
        CPS::Bool checkConvergence();
        /// Logging for integer vectors
        CPS::String logVector(std::vector<CPS::UInt> indexVector) {
            std::stringstream result;
            std::copy(indexVector.begin(), indexVector.end(), std::ostream_iterator<CPS::UInt>(result, " "));
            return result.str().c_str();
        };
        ///
        CPS::Task::List getTasks();
        // determines power flow bus type for each node according to the components attached to it.
    public:
        /// Constructor to be used in simulation examples.
		PFSolver(CPS::String name,
			CPS::SystemTopology system,
			Real timeStep,
			CPS::Logger::Level logLevel);
		///
		virtual ~PFSolver() { };
        
        /// Set a node to VD using its name
        void setVDNode(CPS::String name);
        /// Allows to modify the powerflow bus type of a specific component
        void modifyPowerFlowBusComponent(CPS::String name, CPS::PowerflowBusType powerFlowBusType);

        class SolveTask : public CPS::Task {
		public:
			SolveTask(PFSolver& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			PFSolver& mSolver;
		};
    };
}