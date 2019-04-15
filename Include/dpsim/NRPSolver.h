/*
 * Authors: Santiago Peñate Vera, Jan Dinkelbach
 *
 * Created on 25 of January of 2015, 23:05
 * Copyright (C) 2015 Santiago Peñate Vera
 * Copyright (C) 2019 Jan Dinkelbach
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef SOLVER_NRPOLAR_H
#define	SOLVER_NRPOLAR_H

#include <cmath>

#include <dpsim/Solver.h>
#include <dpsim/Scheduler.h>
#include "cps/SystemTopology.h"
#include "cps/Components.h"
#include "cps/Component.h"
#include <cps/Logger.h>
#include <iterator>



namespace DPsim {

    /*!
     * \brief This class implements the Nerwton-Raphson method of load flow
     *  analysis using polar coordinates.
     */
    class NRpolarSolver: public Solver
    {
	protected:
		CPS::String mName;
		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		/// Simulation logger
		CPS::Logger mLog;
		/// Logging for integer vectors
		CPS::String logVector(std::vector<int> index_vector)
		{
			std::stringstream result;
			std::copy(index_vector.begin(), index_vector.end(), std::ostream_iterator<int>(result, " "));
			return result.str().c_str();
		};
    public:
		// General simulation settings
		/// System time step is constant for NRP solver
		CPS::Real mTimeStep;

		CPS::Real Sbase;

		CPS::SparseMatrixCompRow Y;

		CPS::MatrixComp Z;

		CPS::MatrixComp Zred;

		//solutions
		CPS::UInt sol_length;
		/// Complex solution
		CPS::VectorComp sol_S_complex;
		CPS::VectorComp sol_V_complex;

		/// Cartesian solution
		CPS::Vector sol_P;
		CPS::Vector sol_Q;
		CPS::Vector sol_V;
		CPS::Vector sol_D;
		CPS::Real sol_Vi(CPS::UInt k);
		CPS::Real sol_Vr(CPS::UInt k);
		CPS::Complex sol_Vcx(CPS::UInt k);
		CPS::Complex sol_Scx(CPS::UInt k);

		// solution settings
		bool sol_initialized = false;
		bool sol_complex_initialized = false;
		void resize_sol(int n);
		void resize_complex_sol(int n);
		void clear_sol();
		void clear_complex_sol(int n);
		///Real part of Y
		CPS::Real G(int i, int j);
		///Imaginary part of Y
		CPS::Real B(int i, int j);


		// solver settings
		double tolerance=10e-9;
		/// Maximum number of iterations
		CPS::UInt maxIterations=9;
		CPS::UInt Iterations;

		// Constructor
		NRpolarSolver(CPS::String simName,
			CPS::SystemTopology & sysTopology,
			Real timeStep,
			CPS::Domain domain,
			CPS::Logger::Level logLevel);
		// Destructor
		virtual ~NRpolarSolver() noexcept;

		/*!
		* \brief Solves a polynomial of 3rd degree
		*
		* This method solves a polynomial defined by the coeffients
		* g0, g1, g3 and g3 such that $d + c*x + b*x^2 + a*x^3 = 0$.
		*
		* Provides the real solution using the Newton-Raphson technique
		*/
		double solve3rdDegreePolynomial(
				double d,
				double c,
				double b,
				double a,
			double x) const;

		//! \brief Checks whether a particular solution converged
		bool converged(CPS::Vector const& PQinc, CPS::UInt npqpvpq) const;

		/*!
		* \brief Solves the grid
		*
		* \
		*
		* \sa Solver_State
		*/
		virtual void NRP_initialize();

		virtual void powerFlow();

        void determinePowerFlowBusType();

		void update_solution_power_from_circuit();

		void setSbase();

		void setVDNode(CPS::String name);

		void modifyPowerFlowBusComponent(CPS::String name,CPS::PowerflowBusType powerFlowBusType);

		Real step(Real time);

		CPS::Task::List getTasks();


    private:
		CPS::SystemTopology SysTopology;

        std::vector<int> BUSES;

        std::vector<int> PQPV;

        std::vector<int> LastPQ;

        std::vector<int> LastPV;

		//******* vectors of node indices for pf calculation ********
		std::vector<CPS::UInt> PQBusIndices;
		std::vector<CPS::UInt> PVBusIndices;
		std::vector<CPS::UInt> slackBusIndex;

		//******* vectors of nodes for pf calculation ********
		CPS::TopologicalNode::List PQBuses;
		CPS::TopologicalNode::List PVBuses;
		CPS::TopologicalNode::List slackBus;

        /****** vectors of components ******/
		std::vector<std::shared_ptr<CPS::Static::Ph1::Transformer>> Transformers;
		std::vector<std::shared_ptr<CPS::Static::Ph1::SynchronGenerator>> SynchronGenerators;
		std::vector<std::shared_ptr<CPS::Static::Ph1::Load>> Loads;
		std::vector<std::shared_ptr<CPS::Static::Ph1::PiLine>> Lines;
		std::vector<std::shared_ptr<CPS::Static::Ph1::Shunt>> Shunts;
		std::vector<std::shared_ptr<CPS::Static::Ph1::externalGridInjection>> externalGrids;

		CPS::Vector Pesp;

		CPS::Vector Qesp;

		void Jacobian(CPS::Matrix &J, CPS::Vector &V, CPS::Vector &D, CPS::UInt npq, CPS::UInt npv); //calculate the jacobian, J is passed by reference

		double mu(CPS::Matrix &J, CPS::Matrix &J2, CPS::Vector &F, CPS::Vector &dV, CPS::Vector &dD, CPS::Vector & dx, CPS::UInt npq, CPS::UInt npv);

		void get_power_inc(CPS::Vector &PQinc, CPS::UInt npq, CPS::UInt npv); //PQinc is passed by reference

		void calculate_Q(CPS::UInt npq, CPS::UInt npv); //calculate the reative power at the PV buses

		double Q(CPS::UInt k);

		double P(CPS::UInt k);

		void update_solution(CPS::Vector X, CPS::UInt npq, CPS::UInt npv);

		void get_increments(CPS::Vector X, CPS::Vector &incV, CPS::Vector &incD, CPS::UInt npq, CPS::UInt npv);

		void calculate_slack_power(); //calculate the slack bus power

		/*
		* Composes the circuit admittance matrix
		*/
		void compose_Y();

		void compile();

		void generate_initial_solution(bool keep_last_solution=false);

		void set_solution();
		void calculate_flows();

		/* TODO: Z matrix composition to be fixed after moving from Circuit to Solver_NRpolar
		* Composes the circuit impedance matrix and reduced ipedance matrix by
		* inverting the admittance matrix
		*/
		//void compose_Z();

		//! \brief Checks whether the solver can work on the given model
		bool checks() const;


		void correct_PVbuses_violating_Q(CPS::UInt &npq, CPS::UInt &npv, CPS::Matrix &J, CPS::Vector &K, CPS::Vector &X);

		// TODO proper tasking system integration for parallelism
		class SolveTask : public CPS::Task {
		public:
			SolveTask(NRpolarSolver& solver) :
				Task(solver.mName + ".Solve"), mSolver(solver) {
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			NRpolarSolver& mSolver;
		};
    };





#endif	/* SOLVER_NRPOLAR_H */

}
