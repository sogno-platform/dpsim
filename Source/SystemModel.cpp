/** A simulation model
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

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
}

void SystemModel::addSystemMatrix(Matrix& systemMatrix) {
	mSystemMatrixVector.push_back(systemMatrix);
	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);
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

void SystemModel::addCompToSystemMatrix(Int row, Int column, Complex value) {
	addCompToSystemMatrix(row, column, value.real(), value.imag());
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
