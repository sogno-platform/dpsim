#ifndef SYSTEMMODEL_H
#define SYSTEMMODEL_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Logger.h"

namespace DPsim {

	enum class SimulationType { DynPhasor, EMT };

	class SystemModel {

	private:
		/// Simulation type
		SimulationType mSimType;
		/// Number of nodes
		int mNumNodes;
		/// Index offset for imaginary part
		int mCompOffset;
		/// Angular frequency of the phasor
		Real mSystemOmega;
		/// Simulation time step
		Real mTimeStep;

		
		/// LU decomposition of system matrix A
		Eigen::PartialPivLU<DPSMatrix> mLuFactored;
		/// LU decomposition of system matrix A
		std::vector<Eigen::PartialPivLU<DPSMatrix> > mLuFactoredVector;
		/// System matrix A that is modified by matrix stamps 
		DPSMatrix mSystemMatrix;
		/// System matrices list for swtiching events
		std::vector<DPSMatrix> mSystemMatrixVector;
		/// Vector of known quantities
		DPSMatrix mRightSideVector;
		/// Vector of unknown quantities
		DPSMatrix mLeftSideVector;

	public:
		SystemModel() {}
		void initialize();
		void addSystemMatrix();

		Matrix getCurrentSystemMatrix() { return mSystemMatrix; }
		Matrix getLUdecomp() { return mLuFactored.matrixLU(); }
		Matrix getRightSideVector() { return mRightSideVector; }
		Real getTimeStep() { return mTimeStep; }
		Real getOmega() { return mSystemOmega; }
		Int getCompOffset() { return mCompOffset; }
		Real getRealFromLeftSideVector(Int row) { return mLeftSideVector(row, 0); }
		Real getImagFromLeftSideVector(Int row) { return mLeftSideVector(row + mCompOffset, 0); }

		void addCompToSystemMatrix(Int row, Int column, Real reValue, Real imValue) {
			mSystemMatrix(row, column) = mSystemMatrix(row, column) + reValue; 
			mSystemMatrix(row + mCompOffset, column + mCompOffset) = mSystemMatrix(row + mCompOffset, column + mCompOffset) + reValue;
			mSystemMatrix(row, column + mCompOffset) = mSystemMatrix(row, column + mCompOffset) - imValue;
			mSystemMatrix(row + mCompOffset, column) = mSystemMatrix(row + mCompOffset, column) + imValue;
		}
		
		void addCompToRightSideVector(Int row, Real reValue, Real imValue) {
			mRightSideVector(row, 0) = mRightSideVector(row, 0) + reValue;
			mRightSideVector(row + mCompOffset, 0) = mRightSideVector(row + mCompOffset, 0) + imValue;
		}

		void setSystemMatrixElement(Int row, Int column, Real value) {
			mSystemMatrix(row, column) = value;
		}

		void addToRightSideVector(Int row, Real value) {
			mRightSideVector(row, 0) = value;
		}

		void setSimType(SimulationType simType) { mSimType = simType; }
		void setTimeStep(Real timeStep) { mTimeStep = timeStep; }
		void setOmega(Real omega) { mSystemOmega = omega; }
		void InitializeRightSideVector(DPsim::Matrix& rightSideVector) { mRightSideVector = rightSideVector; }
		void InitializeLeftSideVector(DPsim::Matrix& leftSideVector) { mLeftSideVector = leftSideVector; }
		
		void switchSystemMatrix(int systemMatrixIndex);

	};
}
#endif
