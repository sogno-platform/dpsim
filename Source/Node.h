/** Base component
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
	class Terminal;

	class Node {
	public:
		typedef std::shared_ptr<Node> Ptr;
		typedef std::vector<Ptr> List;
		String mRID;
		String mName;
		Matrix::Index mSimNode;
		Complex mVoltage;
		std::vector<std::weak_ptr<Terminal>> mTerminals;
		Node(Matrix::Index simNode) : mSimNode(simNode) {}
		Node(String rid, Matrix::Index simNode) : mRID(rid), mSimNode(simNode) {}
		std::shared_ptr<Terminal> getTerminal(Int position) {
			if (mTerminals.size() <= position)
				return nullptr;
			return mTerminals[position].lock();
		}
	};
}
