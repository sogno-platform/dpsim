/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
	/// Interface to be used by synchronous generators
	class MNASyncGenInterface {
	protected:
		///
		Attribute<Int>::Ptr mNumIter;
		///
		Int mMaxIter = 25;
		///
		Real mTolerance = 1e-6;

	public:
		typedef std::shared_ptr<MNASyncGenInterface> Ptr;
		typedef std::vector<Ptr> List;

		// Solver functions
		///
		virtual void correctorStep()=0;
		///
		virtual void updateVoltage(const Matrix& leftVector)=0;
		///
		virtual bool requiresIteration() {return false;}

		/// Setters
		///
		void setMaxIterations(Int maxIterations) {mMaxIter = maxIterations;}
		///
		void setTolerance(Real Tolerance) {mTolerance = Tolerance;}

	protected:
		/// Constructor
		MNASyncGenInterface() { }
	};
}
