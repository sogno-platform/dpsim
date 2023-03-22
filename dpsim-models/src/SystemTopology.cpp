/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <unordered_map>

#include <dpsim-models/SystemTopology.h>
#include <dpsim-models/SP/SP_Ph1_SynchronGenerator.h>

using namespace CPS;

Matrix SystemTopology::initFrequency(Real frequency) const {
	Matrix frequencies(1,1);
	frequencies << frequency;
	return frequencies;
}

void SystemTopology::addNode(TopologicalNode::Ptr topNode) {
	if (auto nodeComplex = std::dynamic_pointer_cast<SimNode<Complex>>(topNode)) nodeComplex->initialize(mFrequencies);
	if (auto nodeReal = std::dynamic_pointer_cast<SimNode<Real>>(topNode)) nodeReal->initialize(mFrequencies);

	mNodes.push_back(topNode);
}

void SystemTopology::addNodeAt(TopologicalNode::Ptr topNode, UInt index) {
	if (auto node = std::dynamic_pointer_cast<SimNode<Complex>>(topNode)) node->initialize(mFrequencies);
	if (auto nodeReal = std::dynamic_pointer_cast<SimNode<Real>>(topNode)) nodeReal->initialize(mFrequencies);

	if (index > mNodes.capacity())
		mNodes.resize(index+1);

	mNodes[index] = topNode;
}


void SystemTopology::addNodes(const TopologicalNode::List& topNodes) {
	for (auto topNode : topNodes)
		addNode(topNode);
}

void SystemTopology::addComponent(IdentifiedObject::Ptr component) {
	if (auto powerCompComplex = std::dynamic_pointer_cast<SimPowerComp<Complex>>(component)) powerCompComplex->initialize(mFrequencies);
	if (auto powerCompReal = std::dynamic_pointer_cast<SimPowerComp<Real>>(component)) powerCompReal->initialize(mFrequencies);

	mComponents.push_back(component);
}

template <typename VarType>
void SystemTopology::connectComponentToNodes(typename SimPowerComp<VarType>::Ptr component, typename SimNode<VarType>::List simNodes) {
	component->connect(simNodes);
	for (auto simNode : simNodes)
		mComponentsAtNode[simNode].push_back(component);
}

void SystemTopology::componentsAtNodeList() {
	for (auto comp : mComponents) {
		auto powerComp = std::dynamic_pointer_cast<TopologicalPowerComp>(comp);
		if (powerComp)
			for (auto topoNode : powerComp->topologicalNodes())
				mComponentsAtNode[topoNode].push_back(powerComp);
	}
}

void SystemTopology::addComponents(const IdentifiedObject::List& components) {
	for (auto comp : components)
		addComponent(comp);
}

void SystemTopology::initWithPowerflow(const SystemTopology& systemPF, CPS::Domain domain) {

	for (auto nodePF : systemPF.mNodes) {
		if (auto node = this->node<TopologicalNode>(nodePF->name())) {
			// Initiation of phase B and C in 3 Phase systems ? 
			//SPDLOG_LOGGER_INFO(mSLog, "Updating initial voltage of {} according to powerflow", node->name());
			//SPDLOG_LOGGER_INFO(mSLog, "Former initial voltage: {}", node->initialSingleVoltage());
			node->setInitialVoltage(std::dynamic_pointer_cast<CPS::SimNode<CPS::Complex>>(nodePF)->singleVoltage());
			//SPDLOG_LOGGER_INFO(mSLog, "Updated initial voltage: {}", node->initialSingleVoltage());
		}
	}

	// set initial power of SG
	for (auto compPF : systemPF.mComponents) {
		if (auto genPF = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(compPF)) {
			if (domain==CPS::Domain::DP || domain==CPS::Domain::SP) {
				auto comp = this->component<SimPowerComp<Complex>>(compPF->name());
				auto terminal = comp->terminals()[0];
				terminal->setPower(-genPF->getApparentPower());
			} else if (domain==CPS::Domain::EMT) {
				auto comp = this->component<SimPowerComp<Real>>(compPF->name());
				auto terminal = comp->terminals()[0];
				terminal->setPower(-genPF->getApparentPower());
			}
			//SPDLOG_LOGGER_INFO(mSLog, "Updated initial power of gen {}: {}", compPF->name(), genPF->getApparentPower());
		}
	}
}

void SystemTopology::addTearComponent(IdentifiedObject::Ptr component) {
	if (auto powerCompComplex = std::dynamic_pointer_cast<SimPowerComp<Complex>>(component)) powerCompComplex->initialize(mFrequencies);

	if (auto powerCompReal = std::dynamic_pointer_cast<SimPowerComp<Real>>(component)) powerCompReal->initialize(mFrequencies);

	mTearComponents.push_back(component);
}

void SystemTopology::addTearComponents(const IdentifiedObject::List& components) {
	for (auto comp : components)
		addTearComponent(comp);
}


template<typename Type>
typename std::shared_ptr<Type> SystemTopology::node(UInt index) {
	if (index < mNodes.size()) {
		auto topoNode = mNodes[index];
		auto node = std::dynamic_pointer_cast<Type>(topoNode);
		if (node)
			return node;
	}

	return nullptr;
}

template<typename Type>
typename std::shared_ptr<Type> SystemTopology::node(std::string_view name) {
	for (auto topoNode : mNodes) {
		if (topoNode->name() == name) {
			auto node = std::dynamic_pointer_cast<Type>(topoNode);
			if (node)
				return node;
			else
				return nullptr;
		}
	}
	return nullptr;
}

std::map<String, String, std::less<>> SystemTopology::listIdObjects() const {
	std::map<String, String, std::less<>> objTypeMap;

	for (auto node : mNodes) {
		objTypeMap[node->name()] = node->type();
	}
	for (auto comp : mComponents) {
		objTypeMap[comp->name()] = comp->type();
	}
	return objTypeMap;
}

template<typename VarType>
void SystemTopology::multiplyPowerComps(Int numberCopies) {
	typename SimNode<VarType>::List newNodes;
	typename SimPowerComp<VarType>::List newComponents;

	for (int copy = 0; copy < numberCopies; copy++) {
		std::unordered_map<typename SimNode<VarType>::Ptr, typename SimNode<VarType>::Ptr> nodeMap;
		String copySuffix = "_" + std::to_string(copy+2);

		// copy nodes
		typename SimNode<VarType>::Ptr nodePtr;
		for (size_t nNode = 0; nNode < mNodes.size(); nNode++) {
			auto nodePtr = this->node<SimNode<VarType>>(static_cast<UInt>(nNode));
			if (!nodePtr)
				continue;

			// GND is not copied
			if (nodePtr->isGround()) {
				nodeMap[nodePtr] = nodePtr;
			} else {
				auto nodeCpy = SimNode<VarType>::make(nodePtr->name() + copySuffix, nodePtr->phaseType());
				nodeCpy->setInitialVoltage(nodePtr->initialVoltage());
				nodeMap[nodePtr] = nodeCpy;
				newNodes.push_back(nodeCpy);
			}
		}

		// copy components
		for (auto genComp : mComponents) {
			auto comp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(genComp);
			if (!comp)
				continue;
			auto copy = comp->clone(comp->name() + copySuffix);
			if (!copy)
				throw SystemError("copy() not implemented for " + comp->name());

			// map the nodes to their new copies, creating new terminals
			typename SimNode<VarType>::List nodeCopies;
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
	// SimPowerComps should be all EMT or all DP anyway, but this way we don't have to look
	multiplyPowerComps<Real>(numCopies);
	multiplyPowerComps<Complex>(numCopies);
}

/// DEPRECATED: Unused
void SystemTopology::reset() {
	// for (auto c : mComponents) {
	// 	c->reset();
	// }
}

template <typename VarType>
void SystemTopology::splitSubnets(std::vector<SystemTopology>& splitSystems) {
	std::unordered_map<typename SimNode<VarType>::Ptr, int> subnet;
	int numberSubnets = checkTopologySubnets<VarType>(subnet);
	if (numberSubnets == 1) {
		splitSystems.push_back(*this);
	} else {
		std::vector<IdentifiedObject::List> components(numberSubnets);
		std::vector<TopologicalNode::List> nodes(numberSubnets);

		// Split nodes into subnet groups
		for (auto node : mNodes) {
			auto pnode = std::dynamic_pointer_cast<SimNode<VarType>>(node);
			if (!pnode || node->isGround())
				continue;

			nodes[subnet[pnode]].push_back(node);
		}

		// Split components into subnet groups
		for (auto comp : mComponents) {
			auto pcomp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
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
int SystemTopology::checkTopologySubnets(std::unordered_map<typename SimNode<VarType>::Ptr, int>& subnet) {
	std::unordered_map<typename SimNode<VarType>::Ptr, typename SimNode<VarType>::List> neighbours;

	for (auto comp : mComponents) {
		auto pcomp = std::dynamic_pointer_cast<SimPowerComp<VarType>>(comp);
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
		auto node = std::dynamic_pointer_cast<SimNode<VarType>>(tnode);
		if (!node || tnode->isGround()) {
			totalNodes--;
		}
	}

	while (subnet.size() != totalNodes) {
		std::list<typename SimNode<VarType>::Ptr> nextSet;

		for (auto tnode : mNodes) {
			auto node = std::dynamic_pointer_cast<SimNode<VarType>>(tnode);
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

		label << "<FONT POINT-SIZE=\"12\"><B>" << node->name() << "</B></FONT><BR/>";

		double phase = 180.0 / M_PI * std::arg(node->initialSingleVoltage());
		double mag = std::abs(node->initialSingleVoltage());

		const char *suffixes[] = { "", "k", "M", "G" };

		int s;
		for (s = 0; s < 3 && mag > 1000; s++)
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

		TopologicalPowerComp::Ptr topoComp;

		if (!(topoComp = std::dynamic_pointer_cast<TopologicalPowerComp>(comp)))
			continue;

		c = g.addNode(topoComp->uid());

		auto type = topoComp->type();
		auto name = topoComp->name();

		std::stringstream label, tooltip;

		label << "<FONT POINT-SIZE=\"12\"><B>" << name << "</B></FONT><BR/>";
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

String SystemTopology::render() {
	auto graph = this->topologyGraph();
	std::stringstream ss;
	graph.render(ss, "neato", "svg");

	return ss.str();
}

void SystemTopology::renderToFile(String filename) {
	std::ofstream ofstr(filename);
	this->topologyGraph().render(ofstr, "neato", "svg");
}
#endif

// Explicit instantiation of template functions to be able to keep the definition in the cpp
template void SystemTopology::multiplyPowerComps<Real>(Int numberCopies);
template void SystemTopology::multiplyPowerComps<Complex>(Int numberCopies);
template std::shared_ptr<TopologicalNode> SystemTopology::node<TopologicalNode>(UInt index);
template std::shared_ptr<TopologicalNode> SystemTopology::node<TopologicalNode>(std::string_view name);
template std::shared_ptr<SimNode<Real>> SystemTopology::node<SimNode<Real>>(UInt index);
template std::shared_ptr<SimNode<Complex>> SystemTopology::node<SimNode<Complex>>(UInt index);
template std::shared_ptr<SimNode<Real>> SystemTopology::node<SimNode<Real>>(std::string_view name);
template std::shared_ptr<SimNode<Complex>> SystemTopology::node<SimNode<Complex>>(std::string_view name);
template void SystemTopology::connectComponentToNodes<Real>(typename SimPowerComp<Real>::Ptr component, typename SimNode<Real>::List simNodes);
template void SystemTopology::connectComponentToNodes<Complex>(typename SimPowerComp<Complex>::Ptr component, typename SimNode<Complex>::List simNodes);
template int SystemTopology::checkTopologySubnets<Real>(std::unordered_map<typename CPS::SimNode<Real>::Ptr, int>& subnet);
template int SystemTopology::checkTopologySubnets<Complex>(std::unordered_map<typename CPS::SimNode<Complex>::Ptr, int>& subnet);
template void SystemTopology::splitSubnets<Real>(std::vector<CPS::SystemTopology>& splitSystems);
template void SystemTopology::splitSubnets<Complex>(std::vector<CPS::SystemTopology>& splitSystems);
