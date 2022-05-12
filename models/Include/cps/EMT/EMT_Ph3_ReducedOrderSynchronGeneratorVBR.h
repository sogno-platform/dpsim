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
namespace EMT {
namespace Ph3 {
	/// @brief Base class for EMT VBR simplefied synchronous generator models
	class ReducedOrderSynchronGeneratorVBR :
		public Base::ReducedOrderSynchronGenerator<Real>,
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
        ReducedOrderSynchronGeneratorVBR(String uid, String name, Logger::Level logLevel);
        ReducedOrderSynchronGeneratorVBR(String name, Logger::Level logLevel);
      
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
        void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);

    public:
        virtual ~ReducedOrderSynchronGeneratorVBR();
    
        /// Mark that parameter changes so that system matrix is updated
		Bool hasParameterChanged() override { return 1; };
    };
}
}
}