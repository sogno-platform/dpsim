#include "SystemModel.h"

using namespace DPsim;

void SystemModel::initialize(Int numNodes, Int numIdealVS) {
	mNumNodes = numNodes;
	mNumIdealVS = numIdealVS;
	mCompOffset = mNumNodes;

	if (mSimType == SimulationType::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(mNumNodes, 1);
		mSystemMatrix = DPSMatrix::Zero(mNumNodes, mNumNodes);

	}
	else {
		mRightSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
		mSystemMatrix = DPSMatrix::Zero(2 * mNumNodes, 2 * mNumNodes);

	}

	switchSystemMatrix(0);	
}

void SystemModel::addSystemMatrix(Matrix systemMatrix) {	
	mSystemMatrixVector.push_back(systemMatrix);
	//mSystemMatrix = systemMatrix;

	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);
	mLuFactored = luFactored;
}

void SystemModel::addRealToSystemMatrix(Int row, Int column, Real value) {
	mSystemMatrix(row, column) = mSystemMatrix(row, column) + value;
}

void SystemModel::addCompToSystemMatrix(Int row, Int column, Real reValue, Real imValue) {
	mSystemMatrix(row, column) = mSystemMatrix(row, column) + reValue;
	mSystemMatrix(row + mCompOffset, column + mCompOffset) = mSystemMatrix(row + mCompOffset, column + mCompOffset) + reValue;
	mSystemMatrix(row, column + mCompOffset) = mSystemMatrix(row, column + mCompOffset) - imValue;
	mSystemMatrix(row + mCompOffset, column) = mSystemMatrix(row + mCompOffset, column) + imValue;
}

void SystemModel::addCompToRightSideVector(Int row, Real reValue, Real imValue) {
	mRightSideVector(row, 0) = mRightSideVector(row, 0) + reValue;
	mRightSideVector(row + mCompOffset, 0) = mRightSideVector(row + mCompOffset, 0) + imValue;
}

void SystemModel::addRealToRightSideVector(Int row, Real value) {
	mRightSideVector(row, 0) = value;
}


void SystemModel::solve() {
	mLeftSideVector = mLuFactored.solve(mRightSideVector);
}


void SystemModel::switchSystemMatrix(Int systemMatrixIndex) {
	if (systemMatrixIndex < mSystemMatrixVector.size()) {
		mSystemMatrix = mSystemMatrixVector[systemMatrixIndex];
		mLuFactored = mLuFactoredVector[systemMatrixIndex];
	}
}

void SystemModel::setRightSideVectorToZero() {
	mRightSideVector.setZero();
}
