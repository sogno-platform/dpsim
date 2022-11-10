/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include<vector>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>

namespace CPS {
	class DAEInterface {
	public:
		typedef std::shared_ptr<DAEInterface> Ptr;
		typedef std::vector<Ptr> List;

		using ResFn = std::function<void(double, const double *, const double *, double *, std::vector<int>&)>;

		// DAE Solver: Initial complex Current (of phase a)
		Complex mInitfComplexCurrent = Complex(0,0);
		/// Derivative of the current
		//Matrix mIntfDerCurrent = Matrix::Zero(3,1);
		///
		Real mAbsTolerance = 1e-3;
		///
		void daeSetAbsoluteTolerance(Real AbsTol) {
			mAbsTolerance=AbsTol;
		} 
		/// set init value of the current, calculate and set the 
		/// initial value of the derivative of the current
		void setInitialComplexIntfCurrent(Complex initCurrent) {};
		///
		virtual void daeInitialize(double time, double state[], double dstate_dt[], 
			double absoluteTolerances[], double stateVarTypes[], int& counter)=0;
		///
		virtual void daePreStep(double time)=0;
		///Residual Function for DAE Solver
		virtual void daeResidual(double sim_time, const double state[], const double dstate_dt[], 
			double resid[], std::vector<int>& off) = 0;
		///
		virtual void daePostStep(double Nexttime, const double state[], 
			const double dstate_dt[], int& counter)=0;
		///
		virtual int getNumberOfStateVariables()=0;
	};
}