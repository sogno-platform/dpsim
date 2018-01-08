/** MathLibrary
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

#include "MathLibrary.h"

using namespace DPsim;

void MathLibrary::setCompMatrixElement(Matrix& mat, Int compOffset, Int row, Int column, Complex value) {
	mat(row, column) = value.real();
	mat(row + compOffset, column + compOffset) = value.real();
	mat(row, column + compOffset) = - value.imag();
	mat(row + compOffset, column) = value.imag();
}

void MathLibrary::setCompVectorElement(Matrix& mat, Int compOffset, Int row, Complex value) {
	mat(row, 0) = value.real();
	mat(row + compOffset, 0) = value.imag();
}

void MathLibrary::addCompToMatrixElement(Matrix& mat, Int compOffset, Int row, Int column, Complex value) {
	mat(row, column) = mat(row, column) + value.real();
	mat(row + compOffset, column + compOffset) = mat(row + compOffset, column + compOffset) + value.real();
	mat(row, column + compOffset) = mat(row, column + compOffset) - value.imag();
	mat(row + compOffset, column) = mat(row + compOffset, column) + value.imag();
}

void MathLibrary::addCompToVectorElement(Matrix& mat, Int compOffset, Int row, Complex value) {
	mat(row, 0) = mat(row, 0) + value.real();
	mat(row + compOffset, 0) = mat(row + compOffset, 0) + value.imag();
}

void MathLibrary::addRealToVectorElement(Matrix& mat, Int row, Real reValue)
{
	mat(row, 0) = mat(row, 0) + reValue;
}

void MathLibrary::addRealToMatrixElement(Matrix& mat, Int row, Int column, Real reValue)
{
	mat(row, column) = mat(row, column) + reValue;
}
