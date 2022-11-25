/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/DP/DP_Ph1_SynchronGeneratorVBR.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief Voltage-Behind-Reactance (VBR) implementation
	/// of 6th order synchronous generator model
	/// Anderson - Fouad's model (Milano, Power System modelling and scripting, chapter 15)
	class SynchronGenerator6bOrderVBR :
		public SynchronGeneratorVBR,
		public SharedFactory<SynchronGenerator6bOrderVBR> {
	public:
		// #### Model specific variables ####
		/// voltage behind transient reactance
		const Attribute<Matrix>::Ptr mEdq_t;
		/// voltage behind subtransient reactance
		const Attribute<Matrix>::Ptr mEdq_s;
	protected:
		/// history term of voltage behind the transient reactance
		Matrix mEh_t;
		/// history term of voltage behind the subtransient reactance
		Matrix mEh_s;

		/// Auxiliar VBR constants
		///
		Real mA;
		///
		Real mB;
		///
		Real mAd_t;
		///
		Real mBd_t;
		///
		Real mAq_t;
		///
		Real mBq_t;
		///
		Real mDq_t;
		///
		Real mAd_s;
		///
		Real mAq_s;
		/// 
		Real mBd_s;
		/// 
		Real mBq_s;
		///
		Real mCd_s;
		///
		Real mCq_s;
		///
		Real mDq_s;

	public:
		///
		SynchronGenerator6bOrderVBR(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator6bOrderVBR(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General Functions ####
		/// Initializes component from power flow data
		void specificInitialization();
		///
		void calculateAuxiliarConstants();
		///
		void stepInPerUnit();
		/// Setter 6th order parameters - extending base class setter by logging
		void setOperationalParametersPerUnit(Real nomPower, 
				Real nomVolt, Real nomFreq, Real H, Real Ld, Real Lq, Real L0,
				Real Ld_t, Real Lq_t, Real Td0_t, Real Tq0_t,
				Real Ld_s, Real Lq_s, Real Td0_s, Real Tq0_s);
	};
}
}
}