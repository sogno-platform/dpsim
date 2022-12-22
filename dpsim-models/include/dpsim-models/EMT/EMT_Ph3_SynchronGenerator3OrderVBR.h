/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.h>
#include <dpsim-models/Solver/DAEInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// @brief Voltage-Behind-Reactance (VBR) implementation
	/// of 3rd order synchronous generator model
	class SynchronGenerator3OrderVBR :
		public ReducedOrderSynchronGeneratorVBR,
		public DAEInterface,
		public SharedFactory<SynchronGenerator3OrderVBR> {
	
	public:
		// ### Model specific elements ###
		/// transient voltage
		const Attribute<Matrix>::Ptr mEdq0_t;
	protected:
		/// history term of VBR
		Matrix mEhs_vbr;

	public:
		///
		SynchronGenerator3OrderVBR(const String & uid, const String & name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator3OrderVBR(const String & name, Logger::Level logLevel = Logger::Level::off);

		// #### General Functions ####
		///
		void specificInitialization() final;
		///
		void stepInPerUnit() final;

		// #### DAE Section ####
		/// Derivative of the current
		MatrixVar<Real> mIntfDerCurrent;
		///
		void daeInitialize(double time, double state[], double dstate_dt[], 
			double absoluteTolerances[], double stateVarTypes[], int& offset) override;
		/// Residual function for DAE Solver
		void daeResidual(double time, const double state[], const double dstate_dt[], 
			double resid[], std::vector<int>& off) override;
		/// Calculation of jacobian
		void daeJacobian(double current_time, const double state[], const double dstate_dt[], 
			SUNMatrix jacobian, double cj, std::vector<int>& off) override;
		///
		void daePostStep(double Nexttime, const double state[], 
			const double dstate_dt[], int& offset) override;
		///
		int getNumberOfStateVariables() override { return 5; }
	};
}
}
}
