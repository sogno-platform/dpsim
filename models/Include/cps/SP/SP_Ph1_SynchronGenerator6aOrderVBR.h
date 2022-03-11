/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SP/SP_Ph1_SynchronGeneratorVBR.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Voltage-Behind-Reactance (VBR) implementation
	/// of 6th order synchronous generator model
	/// Marconato's model (Milano, Power System modelling and scripting, chapter 15)
	class SynchronGenerator6aOrderVBR :
		public SynchronGeneratorVBR,
		public SharedFactory<SynchronGenerator6aOrderVBR> {
	protected:
		// ### Model specific elements ###
		/// voltage behind transient reactance
		Matrix mEdq_t;
		/// voltage behind subtransient reactance
		Matrix mEdq_s;
		/// history term of voltage behind the transient reactance
		Matrix mEh_t;
		/// history term of voltage behind the subtransient reactance
		Matrix mEh_s;

		/// Auxiliar VBR constants
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
		///
		Real mYd;
		///
		Real mYq;

	public:
		///
		SynchronGenerator6aOrderVBR(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator6aOrderVBR(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Complex>::Ptr clone(String name);

		// #### General Functions ####
		///
		void specificInitialization();
		///
		void calculateAuxiliarConstants();
		///
		void stepInPerUnit();
	};
}
}
}
