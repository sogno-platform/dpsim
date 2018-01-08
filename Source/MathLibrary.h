/** MathLibrary
*
* @file
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

#pragma once

#include "Definitions.h"

namespace DPsim {

	class MathLibrary {

	public:
		static void setCompMatrixElement(Matrix& mat, Int compOffset, Int row, Int column, Real reValue, Real imValue);
		static void setCompVectorElement(Matrix& mat, Int compOffset, Int row, Real reValue, Real imValue);
		static void addCompToMatrixElement(Matrix& mat, Int compOffset, Int row, Int column, Real reValue, Real imValue);
		static void addCompToVectorElement(Matrix& mat, Int compOffset, Int row, Real reValue, Real imValue);
		static void addRealToVectorElement(Matrix& mat, Int row, Real reValue);
		static void addRealToMatrixElement(Matrix& mat, Int row, Int column, Real reValue);
	};

}

