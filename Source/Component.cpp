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
#include "./Node.h"
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

Component::Component(String name, Int node1, Int node2, Logger::Level logLevel)
	: mLog("Logs/" + name + ".log", logLevel) {
	mName = name;
	mNode1 = node1;
	mNode2 = node2;
	mLogLevel = logLevel;
	mAttributes["name"]  = Attribute<String>::make(&mName, Flags::read);
	mAttributes["node1"] = Attribute<Int>::make(&mNode1, Flags::read);
	mAttributes["node2"] = Attribute<Int>::make(&mNode2, Flags::read);
}

Component::Component(String name, Int node1, Int node2, Int node3, Logger::Level loglevel)
	: Component(name, node1, node2, loglevel) {
	mNode3 = node3;
	mAttributes["node3"] = Attribute<Int>::make(&mNode3, Flags::read);
}

void Component::setVirtualNodeAt(std::shared_ptr<Node> virtualNode, Int nodeNum) {
	if (mNumVirtualNodes <= nodeNum) {
		mLog.Log(Logger::Level::ERROR) << "Virtual Node position number too large for Component " << mName
			<< " - Ignoring" << std::endl;
	}
	mVirtualNodes[nodeNum] = virtualNode;
	mLog.Log(Logger::Level::DEBUG) << "Set virtual Node at position " << nodeNum << " to " << mVirtualNodes[nodeNum]->mName <<
		" with simulation node " << mVirtualNodes[nodeNum]->mSimNode << std::endl;
}

void Component::setTerminals(std::vector<std::shared_ptr<Terminal>> terminals) {
	if (mNumTerminals < terminals.size()) {
		mLog.Log(Logger::Level::ERROR) << "Number of Terminals is too large for Component " << mName
			<< " - Ignoring" << std::endl;
		return;
	}
	mTerminals = terminals;
	for (int i = 0; i < terminals.size(); i++) {
		if (i == 0) mNode1 = terminals[i]->getNode()->mSimNode;
		if (i == 1) mNode2 = terminals[i]->getNode()->mSimNode;
		if (i == 2) mNode3 = terminals[i]->getNode()->mSimNode;
	}
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

	mLog.Log(Logger::Level::DEBUG) << "Set Terminal at position " << terminalPosition << " to Node " << mTerminals[terminalPosition]->getNode()->mName <<
		" with simulation node " << mTerminals[terminalPosition]->getNode()->mSimNode << std::endl;
}

// TODO: process nodes
void Component::setNodes(std::vector<std::shared_ptr<Node>> nodes) {
	if (mNumTerminals < nodes.size()) {
		mLog.Log(Logger::Level::ERROR) << "Number of Nodes is too large for Component "
			<< mName << " - Ignoring" << std::endl;
		return;
	}
	for (int i = 0; i < nodes.size(); i++) {
		String name = mName + "_T" + std::to_string(i);
		std::shared_ptr<Terminal> terminal = std::make_shared<Terminal>(name);
		nodes[i]->mTerminals.push_back(terminal);
		terminal->mNode = nodes[i];
		setTerminalAt(terminal, i);
		terminal->mComponent = weak_from_this();
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
