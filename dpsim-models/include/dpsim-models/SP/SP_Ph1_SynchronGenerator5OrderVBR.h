/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/SP/SP_Ph1_ReducedOrderSynchronGeneratorVBR.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Voltage-Behind-Reactance (VBR) implementation
	/// of 5th type 2 order synchronous generator model
	/// Ref: Milano, Documentation for PSAT version 2.1.6, June 2, 2011, chapter 17.1.2
	class SynchronGenerator5OrderVBR :
		public ReducedOrderSynchronGeneratorVBR,
		public SharedFactory<SynchronGenerator5OrderVBR> {
	public:
		// ### Model specific elements ###
		/// voltage behind transient reactance
		const Attribute<Matrix>::Ptr mEdq_t;
		/// voltage behind subtransient reactance
		const Attribute<Matrix>::Ptr mEdq_s;

	protected:
		/// history term of voltage behind the transient reactance
		Matrix mEh_t;
		/// history term of voltage behind the subtransient reactance
		Matrix mEh_s;

	public:
		///
		SynchronGenerator5OrderVBR(const String & uid, const String & name, Logger::Level logLevel = Logger::Level::off);
		///
		SynchronGenerator5OrderVBR(const String & name, Logger::Level logLevel = Logger::Level::off);

		// #### General Functions ####
		///
		void specificInitialization() override;
		///
		void stepInPerUnit() override;
	};
}
}
}
