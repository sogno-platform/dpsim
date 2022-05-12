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
namespace DP {
namespace Ph1 {
	/// @brief Base class for DP VBR synchronous generator model single phase
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
        Complex mEvbr;
		/// Ka Matrix
		MatrixComp mKa;
		/// Kb Matrix
		MatrixComp mKb;
		/// Kb Matrix
		MatrixComp mKc;
		/// Constant part of Resistance matrix in abc reference frame
		Matrix mResistanceMatrix_const;
		/// phase a equivalent part of mKa
		Complex mKa_1ph;
		/// phase a equivalent part of mKb
		Complex mKb_1ph;
		/// phase a equivalent part of mResistanceMatrix_const
		Complex mR_const_1ph;
		/// Vector to create abc vector from a component
		MatrixComp mShiftVector;
		/// complex conjugate of mShiftVector
		MatrixComp mShiftVectorConj;
		/// Matrix to convert Evbr from dq domain to abc domain (only phase a)
		MatrixComp mKvbr;

        /// Constructor 
        SynchronGeneratorVBR(String uid, String name, Logger::Level logLevel);
        SynchronGeneratorVBR(String name, Logger::Level logLevel);
      
	  	// #### General Functions ####
        /// Specific component initialization
        virtual void specificInitialization()=0;
        ///
        virtual void stepInPerUnit()=0;
		///
        void calculateConductanceMatrix();
		/// Calculate Ka, Kb and Kvbr
		void calculateAuxiliarVariables();
		///
		Matrix get_parkTransformMatrix();        

		// ### MNA Section ###
		void mnaApplySystemMatrixStamp(Matrix& systemMatrix);
        void mnaApplyRightSideVectorStamp(Matrix& rightVector);
		void mnaPostStep(const Matrix& leftVector);
		void mnaInitialize(Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector);

    public:
        virtual ~SynchronGeneratorVBR();

        /// Mark that parameter changes so that system matrix is updated
		Bool hasParameterChanged() override { return 1; };

    };
}
}
}