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
		mCurrentMatrix.resize(mNumNodes, mNumNodes);
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


void SystemModel::resetCurrents() {
	mCurrentMatrix.setZero();
}

void SystemModel::addCurrent(Int node1, Int node2, Complex value) {
	if (node1 >= 0) {
		if (node2 >= 0) {
			mCurrentMatrix.coeffRef(node1, node2) += value;
			mCurrentMatrix.coeffRef(node2, node1) -= value;
		} else {
			mCurrentMatrix.coeffRef(node1, node1) += value;
		}
	} else if (node2 >= 0) {
		mCurrentMatrix.coeffRef(node2, node2) -= value;
	}
}

Complex SystemModel::getCurrent(Int node1, Int node2) {
	if (node1 >= 0) {
		if (node2 >= 0) {
			return mCurrentMatrix.coeffRef(node1, node2);
		}
		return mCurrentMatrix.coeffRef(node1, node1);
	}
	else if (node2 >= 0) {
		return -mCurrentMatrix.coeffRef(node2, node2);
	}
	// shouldn't happen, but return a sensible value anyway
	return Complex(0, 0);
}
