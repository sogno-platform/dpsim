/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	/// \brief MNA interface to be used by switching devices.
	class MNASwitchInterface : public MNAInterface {
	public:
		typedef std::shared_ptr<MNASwitchInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA section ####
		/// Check if switch is closed
		virtual Bool mnaIsClosed() = 0;
		/// Stamps system matrix considering the defined switch position
		virtual void mnaApplySwitchSystemMatrixStamp(Bool closed, Matrix& systemMatrix, Int freqIdx) { }
		/// Stamps (sparse) system matrix considering the defined switch position
		virtual void mnaApplySwitchSystemMatrixStamp(Bool closed, SparseMatrixRow& systemMatrix, Int freqIdx) {
			Matrix mat = Matrix(systemMatrix);
			mnaApplySwitchSystemMatrixStamp(closed, mat, freqIdx);
			systemMatrix = mat.sparseView();
		}
	};
}
