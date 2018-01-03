/** A simulation model
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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
#include "MathLibrary.h"

using namespace DPsim;

void SystemModel::initialize(Int numNodes) {
	mNumNodes = numNodes;
	mCompOffset = mNumNodes;

	if (mSimType == SimulationType::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = Matrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	}
}

void SystemModel::createEmptySystemMatrix() {

	if (mSimType == SimulationType::EMT) {
		mSystemMatrix = Matrix::Zero(mNumNodes, mNumNodes);
	}
	else {
		mSystemMatrix = Matrix::Zero(2 * mNumNodes, 2 * mNumNodes);
	}
}

void SystemModel::addSystemMatrix() {	
	mSystemMatrixVector.push_back(mSystemMatrix);
	mLuFactored = Eigen::PartialPivLU<Matrix>(mSystemMatrix);
	mLuFactoredVector.push_back(mLuFactored);
}

void SystemModel::addRealToSystemMatrix(Int row, Int column, Real value) {
	MathLibrary::addRealToMatrixElement(mSystemMatrix, row, column, value);
}

void SystemModel::addCompToSystemMatrix(Int row, Int column, Real reValue, Real imValue) {
	MathLibrary::addCompToMatrixElement(mSystemMatrix, mCompOffset, row, column, reValue, imValue);
}

void SystemModel::addCompToSystemMatrix(Int row, Int column, Complex value) {
	addCompToSystemMatrix(row, column, value.real(), value.imag());
}

void SystemModel::setCompSystemMatrixElement(Int row, Int column, Real reValue, Real imValue) {
	MathLibrary::setCompMatrixElement(mSystemMatrix, mCompOffset, row, column, reValue, imValue);
}

void SystemModel::addCompToRightSideVector(Int row, Real reValue, Real imValue) {
	MathLibrary::addCompToVectorElement(mRightSideVector, mCompOffset, row, reValue, imValue);
}

void SystemModel::addRealToRightSideVector(Int row, Real value) {
	MathLibrary::addRealToVectorElement(mRightSideVector, row, value);
}

void SystemModel::solve() {
	mLeftSideVector = mLuFactored.solve(mRightSideVector);
}

void SystemModel::switchSystemMatrix(UInt systemMatrixIndex) {
	if (systemMatrixIndex < mSystemMatrixVector.size()) {
		mSystemMatrix = mSystemMatrixVector[systemMatrixIndex];
		mLuFactored = mLuFactoredVector[systemMatrixIndex];
	}
}

void SystemModel::setRightSideVectorToZero() {
	mRightSideVector.setZero();
}


