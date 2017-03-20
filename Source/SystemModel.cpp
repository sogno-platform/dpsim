#include "SystemModel.h"

using namespace DPsim;


void SystemModel::initialize() {
		
	if (mSimType == SimulationType::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
	}

	switchSystemMatrix(0);	
}

void SystemModel::addSystemMatrix() {
	
}


void SystemModel::switchSystemMatrix(int systemMatrixIndex) {
	if (systemMatrixIndex < mSystemMatrixVector.size()) {
		mSystemMatrix = mSystemMatrixVector[systemMatrixIndex];
		mLuFactored = mLuFactoredVector[systemMatrixIndex];
	}
}