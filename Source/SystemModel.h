#ifndef SYSTEMMODEL_H
#define SYSTEMMODEL_H

#include <iostream>
#include <vector>
#include "MathLibrary.h"
#include "Logger.h"

namespace DPsim {

	enum class SimulationType { DynPhasor, EMT };
	enum class NumericalMethod { Euler, AdamBashforth };

	class SystemModel {

	private:
		/// Simulation type
		SimulationType mSimType;
		///Numerical method
		NumericalMethod mNumMethod;
		/// Number of nodes
		int mNumNodes;
		/// Index offset for imaginary part
		int mCompOffset;
		/// Angular frequency of the phasor
		Real mSystemOmega;
		/// Simulation time step
		Real mTimeStep;
		/// Number of ideal Voltage Sources
		Int mNumIdealVS;

		
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

		/** Matrix of all edge currents; diagonal is used for currents to ground. */
		SparseMatrixComp mCurrentMatrix;

	public:
		SystemModel() { }
		void initialize(Int numNodes, Int numIdealVS);
		void addSystemMatrix(Matrix& systemMatrix);

		Matrix & getCurrentSystemMatrix() { return mSystemMatrix; }
		const Matrix & getLUdecomp() { return mLuFactored.matrixLU(); }
		Matrix & getRightSideVector() { return mRightSideVector; }
		Matrix & getLeftSideVector() { return mLeftSideVector; }
		Real getTimeStep() { return mTimeStep; }
		Real getOmega() { return mSystemOmega; }
		Int getCompOffset() { return mCompOffset; }
		Real getRealFromLeftSideVector(Int row) { return mLeftSideVector(row, 0); }
		Real getImagFromLeftSideVector(Int row) { return mLeftSideVector(row + mCompOffset, 0); }
		SimulationType getSimType() { return mSimType; }
		Int getNumNodes() { return mNumNodes; }		
		Int getNumIdealVS() { return mNumIdealVS; }
		NumericalMethod getNumMethod() { return mNumMethod; }
		
		
		void setSimType(SimulationType simType) { mSimType = simType; }
		void setTimeStep(Real timeStep) { mTimeStep = timeStep; }
		void setOmega(Real omega) { mSystemOmega = omega; }
		void setSystemMatrixElement(Int row, Int column, Real value) { mSystemMatrix(row, column) = value; }
		void setNumMethod(NumericalMethod numMethod) { mNumMethod = numMethod; }

		void InitializeRightSideVector(DPsim::Matrix& rightSideVector) { mRightSideVector = rightSideVector; }
		void InitializeLeftSideVector(DPsim::Matrix& leftSideVector) { mLeftSideVector = leftSideVector; }		
		void switchSystemMatrix(Int systemMatrixIndex);
		void addRealToSystemMatrix(Int row, Int column, Real value);
		void addCompToSystemMatrix(Int row, Int column, Real reValue, Real imValue);
		void addCompToRightSideVector(Int row, Real reValue, Real imValue);
		void addRealToRightSideVector(Int row, Real value);
		void setRightSideVectorToZero();

		void solve();
	};
}
#endif
