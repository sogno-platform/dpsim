/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/EMT/EMT_Ph3_SimpSynchronGeneratorVBR.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	class SynchronGenerator3OrderVBR :
		public SimpSynchronGeneratorVBR,
		public SharedFactory<SynchronGenerator3OrderVBR> {
	protected:
		// ### Model specific elements ###
		/// transient voltage
		Matrix mEdq0_t;
		/// history term of VBR
		Matrix mEhs_vbr;
		///
		Real mAq;
		///
		Real mBq;
		///
		Real mCq;

	public:
		///
		SynchronGenerator3OrderVBR(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator3OrderVBR(String name, Logger::Level logLevel = Logger::Level::off);
		///
		SimPowerComp<Real>::Ptr clone(String name);

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