/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Base/Base_PSS.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {

	class PSS1AParameters :
		public Base::PSSParameters,
		public SharedFactory<PSS1AParameters> {
		
		public:
			/// Gain for active power (pu/pu)
			Real Kp;
			/// Gain for bus voltage magnitude (pu/pu)
			Real Kv;
			/// Stabilizer gain (pu/pu)
			Real Kw;
			/// First stabilizer time constant (s)
			Real T1;
			/// Second stabilizer time constant (s)
			Real T2;
			/// Thrid stabilizer time constant (s)
			Real T3;
			/// Fourth stabilizer time constant (s)
			Real T4;
			/// Max stabilizer output signal (pu)
			Real Vs_max;
			/// Min stabilizer output signal (pu)
			Real Vs_min;
			/// Wash-out time constant (s)
			Real Tw;
	};

	/// Power system stabilizer type 2
	/// Ref.: Milano - Power system modelling and scripting, page 371
	class PSS1A :
		public SimSignalComp,
		public Base::PSS,
		public SharedFactory<PSS1A> {

	private: 
		// ### PSS Parameters ####
		///
		std::shared_ptr<PSS1AParameters> mParameters;
		///
		Real mA;
		///
		Real mB;

		/// previos step value of state variable
		///
		Real mV1_prev;
		///
		Real mV2_prev;
		///
		Real mV3_prev;
		/// 
		Real mVs_prev;
		/// 
		Real mOmega_prev;

	protected:
		/// State Variables
		/// Wash-out output
		Real mV1;
		/// Output of the first phase compensation block
		Real mV2;
		/// Output of the second phase compensation block
		Real mV3;
		/// PSS output
		Real mVs;

	public:
		///
		explicit PSS1A(const String & name) : SimSignalComp(name, name) { }
		/// Constructor with log level
		PSS1A(const String & name, CPS::Logger::Level logLevel);
		/// Initializes PSS parameters
		void setParameters(std::shared_ptr<Base::PSSParameters> parameters) final;
		/// Initializes PSS state variables
		void initialize(Real omega, Real activePower, Real Vd, Real Vq) final;
		///
		Real step(Real omega, Real activePower, Real Vd, Real Vq, Real dt) final;
	};
}
}
