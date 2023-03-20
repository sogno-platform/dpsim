/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Definitions.h>

namespace CPS {
namespace DP {
namespace Ph1 {
	/// @brief Class providing interface with DP domain for all components in DQ domain
	class DPDQInterface {

		protected:
			/// Transform from DQ to DP domain
			MatrixFixedSize<2,2> mDQToDPTransform;
			/// Transform from DP to DQ domain
			MatrixFixedSize<2,2> mDPToDQTransform;

			/// Shifting frequency of the DP domain (assumed to be constant)
			Real mOmegaShift;

		public:
			/// Constructor
			DPDQInterface() {}

			/// Setter for shit frequency
			void setDPShiftFrequency(const Real & omegaShift);

			/// Getter for transform from DQ to DP domain
			MatrixFixedSize<2,2> DQToDPTransform() { return mDQToDPTransform; };
			/// Getter for transform from DP to DQ domain
			MatrixFixedSize<2,2> DPToDQTransform() { return mDPToDQTransform; };

			/// Update transformation matrix from DQ to DP
			void updateDQToDPTransform(const Real & thetaDQ, const Real & simTime);
			/// Update transformation matrix from DP to DQ
			void updateDPToDQTransform(const Real & thetaDQ, const Real & simTime);

			/// Apply transform to obtain current complex DP
			Complex applyDQToDPTransform(const MatrixFixedSize<2,1> & dqMatrix);
			/// Apply transform to obtain current DQ vector
			MatrixFixedSize<2,1> applyDPToDQTransform(const Complex& dpComplex);
    };
}
}
}
