/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Base/Base_ReducedOrderSynchronGenerator.h>
#include <cps/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace SP {
namespace Ph1 {
	/// @brief Base class for SP VBR synchronous generator model single phase
	class SynchronGeneratorVBR :
		public Base::ReducedOrderSynchronGenerator<Complex>,
		public MNAVariableCompInterface {
	protected:
        // Common elements of all VBR models
        /// Resistance matrix in dq reference frame
		Matrix mResistanceMatrixDq;
		/// Conductance matrix phase A
		Matrix mConductanceMatrix;
        /// voltage behind reactance phase a
        Complex Evbr;

        /// Park Transformation
		///
		Matrix mDqToComplexA;
		///
		Matrix mComplexAToDq;

        /// Constructor 
        SynchronGeneratorVBR(String uid, String name, Logger::Level logLevel);
        SynchronGeneratorVBR(String name, Logger::Level logLevel);
      
        ///
        virtual void specificInitialization()=0;
        ///
        virtual void stepInPerUnit()=0;
		///
        void calculateResistanceMatrix();
        ///
        Matrix get_DqToComplexATransformMatrix();

        // ### MNA Section ###
        ///
        void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
        void mnaApplyRightSideVectorStamp(Matrix& rightVector);      
        void mnaPostStep(const Matrix& leftVector);
        
    public:
        virtual ~SynchronGeneratorVBR();

        /// Mark that parameter changes so that system matrix is updated
		Bool hasParameterChanged() override { return 1; };
    };
}
}
}
