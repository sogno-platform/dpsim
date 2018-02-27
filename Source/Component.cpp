/** Base component
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
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

#ifndef _MSC_VER
  #include <cxxabi.h>
#endif

#include <typeinfo>

#include "Component.h"
#include "Node.h"
#include "Terminal.h"

using namespace DPsim;

Component::Component(String uid, String name, Logger::Level logLevel)
	: mLog("Logs/" + name + ".log", logLevel) {
	mUID = uid;
	mName = name;
	mLogLevel = logLevel;
}

Component::Component(String name, Logger::Level logLevel)
	: Component(name, name, logLevel) {
}

Component::Component(String name, Matrix::Index node1, Matrix::Index node2, Logger::Level logLevel)
	: mLog("Logs/" + name + ".log", logLevel) {
	mName = name;
	mNode1 = node1;
	mNode2 = node2;
	mLogLevel = logLevel;
	attrMap["name"] = { Attribute::String,  &mName };
	attrMap["node1"] = { Attribute::Integer, &mNode1 };
	attrMap["node2"] = { Attribute::Integer, &mNode2 };
}

Component::Component(String name, Matrix::Index node1, Matrix::Index node2, Matrix::Index node3, Logger::Level loglevel)
	: Component(name, node1, node2, loglevel) {
	mNode3 = node3;
	attrMap["node3"] = { Attribute::Integer, &mNode3 };
}

void Component::setVirtualNodeAt(std::shared_ptr<Node> virtualNode, Int nodeNum) {
	if (mNumVirtualNodes <= nodeNum) {
		mLog.Log(Logger::Level::ERROR) << "Virtual node position number too large for Component " << mName
			<< " - Ignoring" << std::endl;
	}
	mVirtualNodes[nodeNum] = virtualNode;
}

void Component::setTerminals(std::vector<std::shared_ptr<Terminal>> terminals) {
	if (mNumTerminals < terminals.size()) {
		mLog.Log(Logger::Level::ERROR) << "Number of Terminals is too large for Component " << mName
			<< " - Ignoring" << std::endl;
		return;
	}
	mTerminals = terminals;
}

void Component::setTerminalAt(std::shared_ptr<Terminal> terminal, Int terminalPosition) {
	if (mNumTerminals <= terminalPosition) {
		mLog.Log(Logger::Level::ERROR) << "Terminal position number too large for Component " << mName
			<< " - Ignoring" << std::endl;
	}
	mTerminals[terminalPosition] = terminal;
	if (terminalPosition == 0) mNode1 = terminal->getNode()->mSimNode;
	else if (terminalPosition == 1) mNode2 = terminal->getNode()->mSimNode;
	else if (terminalPosition == 2) mNode3 = terminal->getNode()->mSimNode;
}

void Component::setNodes(std::vector<std::shared_ptr<Node>> nodes) {
	if (mNumTerminals < nodes.size()) {
		mLog.Log(Logger::Level::ERROR) << "Number of Nodes is too large for Component " << mName
			<< " - Ignoring" << std::endl;
		return;
	}
}

String Component::getType() {
	Int status = 1;
	const char *mangled;

	mangled = typeid(*this).name();

#ifdef _MSC_VER
	String ret(mangled);

	return ret;
#else
	const char *unmangled;

	unmangled = abi::__cxa_demangle(mangled, NULL, NULL, &status);

	if (status) {
		return mangled;
	}
	else {
		String ret(unmangled);

		free((void *) unmangled);

		if (ret.find("DPsim::") == 0)
			return ret.substr(7);
		else
			return ret;
	}
#endif
}
