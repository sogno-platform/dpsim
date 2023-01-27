/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SimSignalComp.h>
#include <dpsim-models/Logger.h>

namespace CPS {
namespace Signal {
	/// Power system stabilizer type 2
	/// Ref.: Milano - Power system modelling and scripting, page 371
	class PSS1A :
		public SimSignalComp,
		public SharedFactory<PSS1A> {

	private: 
		// ### PSS Parameters ####
		/// Gain for active power (pu/pu)
		Real mKp;
		/// Gain for bus voltage magnitude (pu/pu)
		Real mKv;
		/// Stabilizer gain (pu/pu)
		Real mKw;
		/// First stabilizer time constant (s)
		Real mT1;
		/// Second stabilizer time constant (s)
		Real mT2;
		/// Thrid stabilizer time constant (s)
		Real mT3;
		/// Fourth stabilizer time constant (s)
		Real mT4;
		/// Max stabilizer output signal (pu)
		Real mVs_max;
		/// Min stabilizer output signal (pu)
		Real mVs_min;
		/// Wash-out time constant (s)
		Real mTw;
		/// Simulation step size
		Real mTimeStep;
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
		const Attribute<Real>::Ptr mV1;
		/// Output of the first phase compensation block
		const Attribute<Real>::Ptr mV2;
		/// Output of the second phase compensation block
		const Attribute<Real>::Ptr mV3;
		/// PSS output
		const Attribute<Real>::Ptr mVs;

	public:
		///
		explicit PSS1A(const String & name) : SimSignalComp(name, name) { }
		/// Constructor with log level
		PSS1A(const String & name, CPS::Logger::Level logLevel);
		/// Initializes PSS parameters
		void setParameters(Real Kp, Real Kv, Real Kw, Real T1, 
			Real T2, Real T3, Real T4, Real Vs_max, Real Vs_min, Real Tw, Real dt);
		/// Initializes PSS state variables
		void initialize(Real omega, Real activePower, Real Vd, Real Vq);
		///
		Real step(Real omega, Real activePower, Real Vd, Real Vq);
	};
}
}
