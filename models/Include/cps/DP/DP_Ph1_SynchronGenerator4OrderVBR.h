/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/DP/DP_Ph1_SynchronGeneratorVBR.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief Voltage-Behind-Reactance (VBR) implementation
	/// of 4th order synchronous generator model
	class SynchronGenerator4OrderVBR :
		public SynchronGeneratorVBR,
		public SharedFactory<SynchronGenerator4OrderVBR> {
	protected:
		// #### Model specific variables ####
		/// voltage behind transient reactance
		Matrix mEdq_t;
		/// history term of voltage behind the transient reactance
		Matrix mEh_vbr;

		/// Auxiliar VBR constants
		///
		Real mA;
		///
		Real mB;
		///
		Real mAd;
		///
		Real mAq;
		/// 
		Real mBd;
		///
		Real mBq;
		///
		Real mCq;

	public:
		///
		SynchronGenerator4OrderVBR(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator4OrderVBR(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General Functions ####
		/// Initializes component from power flow data
		void specificInitialization();
		///
		void calculateAuxiliarConstants();
		///
		void stepInPerUnit();
		/// Setter 4th order parameters - extending base class setter by logging
		void setOperationalParametersPerUnit(Real nomPower, 
			Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
			Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t);
	};
}
}
}
