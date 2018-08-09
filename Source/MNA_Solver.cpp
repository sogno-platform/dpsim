/** MNA Solver
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

#include "MNA_Solver.h"

using namespace DPsim;

/// Create left and right side vector
template<>
void MnaSolver<Real>::createEmptyVectors() {	
    mRightSideVector = Matrix::Zero(mNumSimNodes, 1);
    mLeftSideVector = Matrix::Zero(mNumSimNodes, 1);			
}

/// Create left and right side vector
template<>
void MnaSolver<Complex>::createEmptyVectors() {			
    mRightSideVector = Matrix::Zero(2 * mNumSimNodes, 1);
    mLeftSideVector = Matrix::Zero(2 * mNumSimNodes, 1);
}

/// Create system matrix
template<>
void MnaSolver<Real>::createEmptySystemMatrix() {			
    mSystemMatrices.push_back(Matrix::Zero(mNumSimNodes, mNumSimNodes));
}

template<>
void MnaSolver<Complex>::createEmptySystemMatrix() {
    mSystemMatrices.push_back(Matrix::Zero(2 * mNumSimNodes, 2 * mNumSimNodes));			
}

template class DPsim::MnaSolver<Real>;
template class DPsim::MnaSolver<Complex>;