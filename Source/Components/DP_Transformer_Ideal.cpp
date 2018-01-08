/** Ideal Transformer DP
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

#include "DP_Transformer_Ideal.h"

using namespace DPsim;

Components::DP::TransformerIdeal::TransformerIdeal(String name, Int node1, Int node2, Real ratioAbs, Real ratioPhase)
	: Base(name, node1, node2)
{
	mNumVirtualNodes = 1;
	mVirtualNodes = { 0 };
	mRatio = std::polar(ratioAbs, ratioPhase);
}

void Components::DP::TransformerIdeal::applySystemMatrixStamp(SystemModel& system)
{
	if (mNode1 >= 0) {
		system.setCompSystemMatrixElement(mNode1, mVirtualNodes[0], -1.0, 0);
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode1, 1.0, 0);
	}
	if (mNode2 >= 0) {
		system.setCompSystemMatrixElement(mNode2, mVirtualNodes[0], mRatio.real(), mRatio.imag());
		system.setCompSystemMatrixElement(mVirtualNodes[0], mNode2, -mRatio.real(), -mRatio.imag());
	}
}
