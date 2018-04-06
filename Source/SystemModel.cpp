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

using namespace DPsim;

void SystemModel::initialize(Int numNodes) {
	mNumNodes = numNodes;
	mCompOffset = mNumNodes;

	if (mDomain == SimulationType::EMT) {
		mRightSideVector = Matrix::Zero(mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = Matrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = Matrix::Zero(2 * mNumNodes, 1);
	}
}

void SystemModel::createEmptySystemMatrix()
{
	if (mDomain == SimulationType::EMT) {
		mSystemMatrix = Matrix::Zero(mNumNodes, mNumNodes);
	}
	else {
		mSystemMatrix = Matrix::Zero(2 * mNumNodes, 2 * mNumNodes);
	}
}

void SystemModel::addSystemMatrix()
{
	mSystemMatrixVector.push_back(mSystemMatrix);
	mLuFactored = Eigen::PartialPivLU<Matrix>(mSystemMatrix);
	mLuFactoredVector.push_back(mLuFactored);
}

void SystemModel::updateLuFactored()
{		
		mLuFactored = Eigen::PartialPivLU<Matrix>(mSystemMatrix);
}

void SystemModel::addRealToSystemMatrix(Matrix::Index row, Matrix::Index column, Real value)
{
	Math::addRealToMatrixElement(mSystemMatrix, row, column, value);
}

void SystemModel::addCompToSystemMatrix(Matrix::Index row, Matrix::Index column, Complex value) {
	Math::addCompToMatrixElement(mSystemMatrix, row, column, value);
}

void SystemModel::setCompSystemMatrixElement(Matrix::Index row, Matrix::Index column, Complex value) {
	Math::setCompMatrixElement(mSystemMatrix, row, column, value);
}

void SystemModel::addRealToRightSideVector(Matrix::Index row, Real value)
{
	Math::addRealToVectorElement(mRightSideVector, row, value);
}

void SystemModel::addCompToRightSideVector(Matrix::Index row, Complex value) {
	Math::addCompToVectorElement(mRightSideVector, row, value);
}

void SystemModel::solve() {
	mLeftSideVector = mLuFactored.solve(mRightSideVector);
}

void SystemModel::switchSystemMatrix(UInt systemMatrixIndex)
{
	if (systemMatrixIndex < mSystemMatrixVector.size()) {
		mSystemMatrix = mSystemMatrixVector[systemMatrixIndex];
		mLuFactored = mLuFactoredVector[systemMatrixIndex];
	}
}

void SystemModel::setRightSideVectorToZero()
{
	mRightSideVector.setZero();
}
