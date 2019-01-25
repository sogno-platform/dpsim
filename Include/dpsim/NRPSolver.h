/*!
 * \file Solver_NRpolar.h
 * \author Santiago Peñate Vera
 *
 * Created on 25 of January of 2015, 23:05
 * Copyright (C) 2015 Santiago Peñate Vera
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef SOLVER_NRPOLAR_H
#define	SOLVER_NRPOLAR_H

#include <cmath>

#include <dpsim/Solver.h>
#include "NRPSolution.h"
#include "cps/SystemTopology.h"
#include "cps/Components.h"
#include "cps/Component.h"
#include <cps/Logger.h>
#include <iterator>

#define DEFAULT_SOLUTION_TOLERANCE 10e-9
#define DEFAULT_MAX_ITERATIONS 10

using namespace std;

namespace DPsim {

    /*!
     * \brief This class implements the Nerwton-Raphson method of load flow
     *  analysis using polar coordinates.
     */
    class NRpolarSolver: public Solver
    {
    public:
		CPS::Real mTimeStep;

		CPS::Real Sbase;

		CPS::SparseMatrixCompRow Y;

		CPS::MatrixComp Z;

		CPS::MatrixComp Zred;

        CPS::String Slack_mRID;

        CPS::Real Slack_simNode;

		/*Real part of Y
		*/
		CPS::Real G(int i, int j);

		/*Imaginary part of Y
		*/
		CPS::Real B(int i, int j);

		/// Constructor
		NRpolarSolver(CPS::String simName,
			CPS::SystemTopology & sysTopology, 
			Real timeStep,
			CPS::Domain domain,
			CPS::Logger::Level logLevel);

		/// Destructor
		virtual ~NRpolarSolver() noexcept;


		/*!
		* \brief Allowable tolerance of the solver instance
		*
		* \sa DEFAULT_SOLUTION_TOLERANCE
		*/
		double tolerance;


		/*!
		* \brief Maximum number of iterations
		*
		* \sa DEFAULT_MAX_ITERATIONS
		*/
		unsigned maxIterations;
		unsigned Iterations;

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
		bool converged(vec const& PQinc, uint npqpvpq) const;

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

		solution get_initial_solution();

		void update_solution_power_from_circuit();

		void setSbase();

		void setVDNode(CPS::String name);

		void modifyPowerFlowBusComponent(CPS::String name,CPS::PowerflowBusType powerFlowBusType);

		Real step(Real time);

	protected:

		/// Simulation log level
		CPS::Logger::Level mLogLevel;
		/// Simulation logger
		CPS::Logger mLog;
		/// Logging for integer vectors
		CPS::String logVector(vector<int> index_vector)
		{
			std::stringstream result;
			std::copy(index_vector.begin(), index_vector.end(), std::ostream_iterator<int>(result, " "));
			return result.str().c_str();
		};
    private:
		CPS::SystemTopology SysTopology;

        vector<int> BUSES;

        vector<int> PQPV;

        vector<int> LastPQ;
        
        vector<int> LastPV;

		//******* vectors of node indices for pf calculation ********
		std::vector<unsigned int> PQBusIndices;
		std::vector<unsigned int> PVBusIndices;
		std::vector<unsigned int> slackBusIndex;
		
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


		vec Pesp;

		vec Qesp;

		solution Sol;
		cx_solution cx_Sol;

			void Jacobian(mat &J, vec &V, vec &D, uint npq, uint npv); //calculate the jacobian, J is passed by reference
        
			double mu(mat &J, mat &J2, vec &F, vec &dV, vec &dD, vec & dx, uint npq, uint npv);
			void get_power_inc(vec &PQinc, uint npq, uint npv); //PQinc is passed by reference

			void calculate_Q(uint npq, uint npv); //calculate the reative power at the PV buses

			double Q(uint k);

		double P(uint k);

			void update_solution(vec X, uint npq, uint npv);
        
			void get_increments(vec X, vec &incV, vec &incD, uint npq, uint npv);

			void calculate_slack_power(); //calculate the slack bus power        
				      
		/*
		* Composes the circuit admittance matrix
		*/
		void compose_Y();

		void compile();

		void generate_initial_solution(bool keep_last_solution=false);

		void set_solution(solution sol);
		void calculate_flows(cx_solution sol);

		/* TODO: Z matrix composition to be fixed after moving from Circuit to Solver_NRpolar
		* Composes the circuit impedance matrix and reduced ipedance matrix by
		* inverting the admittance matrix
		*/
		//void compose_Z();

		//! \brief Checks whether the solver can work on the given model
		bool checks() const;


		void correct_PVbuses_violating_Q(uint &npq, uint &npv, mat &J, vec &K, vec &X);
        

    };





#endif	/* SOLVER_NRPOLAR_H */

}
