/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/Base/Base_SimpSynchronousGenerator.h>
#include <cps/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph3 {
	/// @brief Base class for EMT VBR simplefied synchronous generator models
	class SimpSynchronGeneratorVBR :
		public Base::SimpSynchronousGenerator<Real>,
		public MNAVariableCompInterface {
	protected:
        // Common elements of all VBR models
        /// Resistance matrix in dq0 reference frame
		Matrix mResistanceMatrixDq0;
		/// Conductance matrix
		Matrix mConductanceMatrix;
        /// voltage behind reactance
        Matrix mEvbr;

		///
		Matrix mAbcToDq0;
		Matrix mDq0ToAbc;

        /// Constructor 
        SimpSynchronGeneratorVBR(String uid, String name, Logger::Level logLevel);
        SimpSynchronGeneratorVBR(String name, Logger::Level logLevel);
      
	  	// #### General Functions ####
        /// Specific component initialization
        virtual void specificInitialization()=0; 
        ///
        virtual void stepInPerUnit()=0;
		///
        void calculateResistanceMatrix();
        /// Park Transformation according to Kundur
        Matrix get_parkTransformMatrix();
		/// Inverse Park Transformation according to Kundur
		Matrix get_inverseParkTransformMatrix();
		
        // ### MNA Section ###
        void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
        void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaPostStep(const Matrix& leftVector);

    public:
        virtual ~SimpSynchronGeneratorVBR();
    
        /// Mark that parameter changes so that system matrix is updated
		Bool hasParameterChanged() override { return 1; };
    };
}
}
}