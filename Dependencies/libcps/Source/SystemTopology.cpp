/**
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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

#include <iostream>
#include <iomanip>
#include <fstream>
#include <unordered_map>

#include <cps/SystemTopology.h>

using namespace CPS;

template<typename VarType>
void SystemTopology::multiplyPowerComps(Int numberCopies) {
	typename Node<VarType>::List newNodes;
	typename PowerComponent<VarType>::List newComponents;

	for (int copy = 0; copy < numberCopies; copy++) {
		std::unordered_map<typename Node<VarType>::Ptr, typename Node<VarType>::Ptr> nodeMap;
		String copySuffix = "_" + std::to_string(copy+2);

		// copy nodes
		typename Node<VarType>::Ptr nodePtr;
		for (size_t nNode = 0; nNode < mNodes.size(); nNode++) {
			auto nodePtr = this->node<Node<VarType>>(static_cast<UInt>(nNode));
			if (!nodePtr)
				continue;

			// GND is not copied
			if (nodePtr->isGround()) {
				nodeMap[nodePtr] = nodePtr;
			} else {
				auto nodeCpy = Node<VarType>::make(nodePtr->name() + copySuffix, nodePtr->phaseType());
				nodeCpy->setInitialVoltage(nodePtr->initialVoltage());
				nodeMap[nodePtr] = nodeCpy;
				newNodes.push_back(nodeCpy);
			}
		}

		// copy components
		for (auto genComp : mComponents) {
			auto comp = std::dynamic_pointer_cast<PowerComponent<VarType>>(genComp);
			if (!comp)
				continue;
			auto copy = comp->clone(comp->name() + copySuffix);
			if (!copy)
				throw SystemError("copy() not implemented for " + comp->name());

			// map the nodes to their new copies, creating new terminals
			typename Node<VarType>::List nodeCopies;
			for (UInt nNode = 0; nNode < comp->terminalNumber(); nNode++) {
				nodeCopies.push_back(nodeMap[comp->node(nNode)]);
			}
			copy->connect(nodeCopies);

			// update the terminal powers for powerflow initialization
			for (UInt nTerminal = 0; nTerminal < comp->terminalNumber(); nTerminal++) {
				copy->terminal(nTerminal)->setPower(comp->terminal(nTerminal)->power());
			}
			newComponents.push_back(copy);
		}
	}
	for (auto node : newNodes)
		addNode(node);
	for (auto comp : newComponents)
		addComponent(comp);
}

void SystemTopology::multiply(Int numCopies) {
	// PowerComponents should be all EMT or all DP anyway, but this way we don't have to look
	multiplyPowerComps<Real>(numCopies);
	multiplyPowerComps<Complex>(numCopies);
}

void SystemTopology::reset() {
	for (auto c : mComponents) {
		c->reset();
	}
}

template <typename VarType>
void SystemTopology::splitSubnets(std::vector<SystemTopology>& splitSystems) {
	std::unordered_map<typename Node<VarType>::Ptr, int> subnet;
	int numberSubnets = checkTopologySubnets<VarType>(subnet);
	if (numberSubnets == 1) {
		splitSystems.push_back(*this);
	} else {
		std::vector<Component::List> components(numberSubnets);
		std::vector<TopologicalNode::List> nodes(numberSubnets);

		// Split nodes into subnet groups
		for (auto node : mNodes) {
			auto pnode = std::dynamic_pointer_cast<Node<VarType>>(node);
			if (!pnode || node->isGround())
				continue;

			nodes[subnet[pnode]].push_back(node);
		}

		// Split components into subnet groups
		for (auto comp : mComponents) {
			auto pcomp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
			if (!pcomp) {
				// TODO this should only be signal components.
				// Proper solution would be to pass them to a different "solver"
				// since they are actually independent of which solver we use
				// for the electric part.
				// Just adding them to an arbitrary solver for now has the same effect.
				components[0].push_back(comp);
				continue;
			}
			for (UInt nodeIdx = 0; nodeIdx < pcomp->terminalNumber(); nodeIdx++) {
				if (!pcomp->node(nodeIdx)->isGround()) {
					components[subnet[pcomp->node(nodeIdx)]].push_back(comp);
					break;
				}
			}
		}
		for (int currentNet = 0; currentNet < numberSubnets; currentNet++) {
			splitSystems.emplace_back(mSystemFrequency,
				nodes[currentNet], components[currentNet]);
		}
	}
}

template <typename VarType>
int SystemTopology::checkTopologySubnets(std::unordered_map<typename Node<VarType>::Ptr, int>& subnet) {
	std::unordered_map<typename Node<VarType>::Ptr, typename Node<VarType>::List> neighbours;

	for (auto comp : mComponents) {
		auto pcomp = std::dynamic_pointer_cast<PowerComponent<VarType>>(comp);
		if (!pcomp)
			continue;

		for (UInt nodeIdx1 = 0; nodeIdx1 < pcomp->terminalNumberConnected(); nodeIdx1++) {
			for (UInt nodeIdx2 = 0; nodeIdx2 < nodeIdx1; nodeIdx2++) {
				auto node1 = pcomp->node(nodeIdx1);
				auto node2 = pcomp->node(nodeIdx2);
				if (node1->isGround() || node2->isGround())
					continue;

				neighbours[node1].push_back(node2);
				neighbours[node2].push_back(node1);
			}
		}
	}

	int currentNet = 0;
	size_t totalNodes = mNodes.size();
	for (auto tnode : mNodes) {
		auto node = std::dynamic_pointer_cast<Node<VarType>>(tnode);
		if (!node || tnode->isGround()) {
			totalNodes--;
		}
	}

	while (subnet.size() != totalNodes) {
		std::list<typename Node<VarType>::Ptr> nextSet;

		for (auto tnode : mNodes) {
			auto node = std::dynamic_pointer_cast<Node<VarType>>(tnode);
			if (!node || tnode->isGround())
				continue;

			if (subnet.find(node) == subnet.end()) {
				nextSet.push_back(node);
				break;
			}
		}
		while (!nextSet.empty()) {
			auto node = nextSet.front();
			nextSet.pop_front();

			subnet[node] = currentNet;
			for (auto neighbour : neighbours[node]) {
				if (subnet.find(neighbour) == subnet.end())
					nextSet.push_back(neighbour);
			}
		}
		currentNet++;
	}
	return currentNet;
}

#ifdef WITH_GRAPHVIZ

Graph::Graph SystemTopology::topologyGraph() {
	Graph::Node *n, *c;

	Graph::Graph g("topology", Graph::Type::undirected);

	g.set("splines", "polyline");

	for (auto node : mNodes) {
		n = g.addNode(node->uid());

		std::stringstream label, tooltip;

		tooltip << node->uid();

		label << node->name() << "<BR/>";

		double phase = 180.0 / M_PI * std::arg(node->initialSingleVoltage());
		double mag = std::abs(node->initialSingleVoltage());

		const char *suffixes[] = { "", "k", "M", "G" };

		int s;
		for (s = 0; s < 4 && mag > 1000; s++)
			mag *= 1e-3;

		if (node->initialSingleVoltage() != Complex(0, 0)) {
			label << std::setprecision(2) << std::fixed;
			label << "<FONT POINT-SIZE=\"10\" COLOR=\"gray28\">";
			label << "(" << mag << " " << suffixes[s] << "V &gt; " << phase << "°)";
			label << "</FONT>";
		}

		n->set("xlabel", label.str(), true);
		n->set("tooltip", tooltip.str(), true);
		n->set("fillcolor", phase == 0 ? "red" : "black");
		n->set("fixedsize", "true");
		n->set("width", "0.15");
		n->set("height", "0.15");
		n->set("shape", "point");
	}

	std::map<String, String> compColorMap;

	for (auto comp : mComponents) {
		if (!comp) // TODO: this is a bug in the CIM::Reader!
			continue;

		TopologicalComponent::Ptr topoComp;

		if (!(topoComp = std::dynamic_pointer_cast<TopologicalComponent>(comp)))
			continue;

		c = g.addNode(topoComp->uid());

		auto type = topoComp->type();
		auto name = topoComp->name();

		std::stringstream label, tooltip;

		label << "<B>" << name << "</B><BR/>";
		label << "<FONT POINT-SIZE=\"10\" COLOR=\"gray28\">" << type << "</FONT>";

		tooltip << "Attributes:";
		for (auto it : topoComp->attributes()) {
			tooltip << std::endl << it.first << ": " << it.second->toString();
		}

		if (compColorMap.find(type) != compColorMap.end()) {
			compColorMap[type] = String("/paired9/") + std::to_string(1 + compColorMap.size() % 9);
		}

		c->set("color", compColorMap[type]);
		c->set("label", label.str(), true);
		c->set("tooltip", tooltip.str(), true);
		c->set("style", "rounded,filled,bold");

		if (type.find("Line") == std::string::npos) {
			c->set("shape", "rectangle");
			c->set("fillcolor", "gray93");
		}
		else {
			c->set("shape", "plaintext");
			c->set("fillcolor", "transparent");
		}

		for (auto term : topoComp->topologicalTerminals()) {
			n = g.node(term->topologicalNodes()->uid());
			if (!n)
				continue;

			g.addEdge(term->uid(), c, n);
		}
	}

	return g;
}
#endif

// Explicit instantiation of template functions to be able to keep the definition in the cpp
template int SystemTopology::checkTopologySubnets<Real>(std::unordered_map<typename CPS::Node<Real>::Ptr, int>& subnet);
template int SystemTopology::checkTopologySubnets<Complex>(std::unordered_map<typename CPS::Node<Complex>::Ptr, int>& subnet);
template void SystemTopology::splitSubnets<Real>(std::vector<CPS::SystemTopology>& splitSystems);
template void SystemTopology::splitSubnets<Complex>(std::vector<CPS::SystemTopology>& splitSystems);
