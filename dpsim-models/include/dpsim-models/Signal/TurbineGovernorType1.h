/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_Governor.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

	class TurbineGovernorType1Parameters :
		public Base::GovernorParameters,
		public SharedFactory<TurbineGovernorType1Parameters> {
		
		public:
			/// Maximum turbine power
			Real Pmax;
			/// Minimum turbine power
			Real Pmin;
			/// Droop
			Real R;
			/// Transient gain time constant (s)
			Real T3;
			/// Power fraction time constant (s)
			Real T4;
			/// Reheat time constant (s)
			Real T5;
			/// Servo time constant (s)
			Real Tc;
			/// Governor time constant (s)
			Real Ts;
			/// Speed reference (pu)
			Real OmRef;
	};

	/// Turbine governor type 1 model 
	/// Ref.: Milano - Power system modelling and scripting, page 358
	class TurbineGovernorType1 :
		public SimSignalComp,
		public Base::Governor,
		public SharedFactory<TurbineGovernorType1> {
	private:
		std::shared_ptr<TurbineGovernorType1Parameters> mParameters;

		// ### Variables ###
		/// Mechanical speed at time k-1
		Real mOmega_prev;
		/// Governor output at time k-1
		Real mXg1_prev;
		/// Servo output at time k-1
		Real mXg2_prev;
		/// Reheat output at time k-1
		Real mXg3_prev;

		// ### Variables ###
		/// Reference torque (pu)
		Real mPmRef;
		/// Governor output at time k
 		Real mXg1;
		/// Servo output at time k
 		Real mXg2;
		/// Reheat output at time k
		Real mXg3;
		/// Mechanical Torque in pu (output of steam turbine)
		Real mTm;

	public:
		/// 
		explicit TurbineGovernorType1(const String & name) : SimSignalComp(name, name) { }

		/// Constructor with log level
		TurbineGovernorType1(const String & name, CPS::Logger::Level logLevel);

		/// Initializes exciter parameters
		void setParameters(std::shared_ptr<Base::GovernorParameters> parameters) final;
		///
		void initialize(Real PmRef) final;
		/// Performs an step to update field voltage value
		Real step(Real Omega, Real dt) final;
	};
}
}
