/** Ideal Transformer DP
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

#include "IdealTransformerDP.h"

using namespace DPsim;

IdealTransformerDP::IdealTransformerDP(String name, Int node1, Int node2, Real ratioRe, Real ratioIm) : BaseComponent(name, node1, node2) {
	mRatioRe = ratioRe;
	mRatioIm = ratioIm;
}

void IdealTransformerDP::applySystemMatrixStamp(SystemModel& system) {
	if (mNode1 >= 0) {
		system.setCompSystemMatrixElement(mNode1, mVirtualNode, -1, 0);
		system.setCompSystemMatrixElement(mVirtualNode, mNode1, 1, 0);
	}
	if (mNode2 >= 0) {
		system.setCompSystemMatrixElement(mNode1, mVirtualNode, mRatioRe, mRatioIm);
		system.setCompSystemMatrixElement(mVirtualNode, mNode1, -mRatioRe, -mRatioIm);
	}
}
