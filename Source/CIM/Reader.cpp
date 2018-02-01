/** Read CIM files
 *
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <CIMModel.hpp>
#include <IEC61970.hpp>
#include "../Definitions.h"
#include "Reader.h"
#include "Components.h"

using namespace DPsim;
using namespace DPsim::CIM;
using namespace IEC61970::Base::Core;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Equivalents;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;


Real Reader::unitValue(Real value, UnitMultiplier mult) {
	switch (mult) {
	case UnitMultiplier::p:
		value *= 1e-12;
		break;
	case UnitMultiplier::n:
		value *= 1e-9;
		break;
	case UnitMultiplier::micro:
		value *= 1e-6;
		break;
	case UnitMultiplier::m:
		value *= 1e-3;
		break;
	case UnitMultiplier::c:
		value *= 1e-2;
		break;
	case UnitMultiplier::d:
		value *= 1e-1;
		break;
	case UnitMultiplier::k:
		value *= 1e3;
		break;
	case UnitMultiplier::M:
		value *= 1e6;
		break;
	case UnitMultiplier::G:
		value *= 1e9;
		break;
	case UnitMultiplier::T:
		value *= 1e12;
		break;
	default:
		break;
	}
	return value;
}

// TODO: fix error with frequency and angular frequency
Reader::Reader(Real systemFrequency, Logger::Level logLevel) : mLog("Logs/CIMpp.log", logLevel) {
	mModel.setDependencyCheckOff();
	mVoltages = nullptr;
	mFrequency = systemFrequency;
}

Reader::~Reader() {
	if (mVoltages)
		delete[] mVoltages;
}

Component::Ptr Reader::mapComponent(BaseClass* obj) {
	if (ACLineSegment *line = dynamic_cast<ACLineSegment*>(obj))
		return mapACLineSegment(line);
	if (ExternalNetworkInjection *inj = dynamic_cast<ExternalNetworkInjection*>(obj))
		return mapExternalNetworkInjection(inj);
	if (EnergyConsumer *consumer = dynamic_cast<EnergyConsumer*>(obj))
		return mapEnergyConsumer(consumer);
	if (PowerTransformer *trans = dynamic_cast<PowerTransformer*>(obj))
		return mapPowerTransformer(trans);
	if (SynchronousMachine *syncMachine = dynamic_cast<SynchronousMachine*>(obj))
		return mapSynchronousMachine(syncMachine);
	return nullptr;
}

bool Reader::addFile(String filename) {
	return mModel.addCIMFile(filename);
}

void Reader::parseFiles() {
	mModel.parseFiles();	
	mLog.Log(Logger::Level::INFO) << "#### List of topological nodes and associated terminals ####" << std::endl;

	for (auto obj : mModel.Objects) {
		TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj);
		if (topNode) {
			// Add this node to global node list and assign simulation node incrementally.
			mPowerflowNodes[topNode->mRID] = std::make_shared<PowerflowNode>(topNode->mRID, (Matrix::Index) mPowerflowNodes.size());

			mLog.Log(Logger::Level::INFO) << "TopologicalNode " << topNode->mRID
				<< "as simulation node " << mPowerflowNodes[topNode->mRID]->mRID << std::endl;			

			for (auto term : topNode->Terminal) {
				// Insert Terminal if it does not exist in the map and add reference to node.
				// This could be optimized because the Terminal is searched twice.
				mPowerflowTerminals.insert(std::make_pair(term->mRID, std::make_shared<PowerflowTerminal>(term->mRID)));
				mPowerflowTerminals[term->mRID]->mNode = mPowerflowNodes[topNode->mRID];
				// Add reference to Terminal to this node.
				mPowerflowNodes[topNode->mRID]->mTerminals.push_back(mPowerflowTerminals[term->mRID]);				
				mLog.Log(Logger::Level::INFO) << "    " << "Terminal " << term->mRID
					<< ", sequenceNumber " << term->sequenceNumber << std::endl;

				// Try to process Equipment connected to Terminal.
				ConductingEquipment *equipment = term->ConductingEquipment;
				if (!equipment) {
					mLog.Log(Logger::Level::WARN) << "Terminal " << term->mRID
						<< " has no Equipment, ignoring!" << std::endl;
				}
				else {
					// Insert Equipment if it does not exist in the map and add reference to Terminal.
					// This could be optimized because the Equipment is searched twice.
					mPowerflowEquipment.insert(std::make_pair(equipment->mRID, std::make_shared<PowerflowEquipment>(equipment->mRID)));
					auto pfEquipment = mPowerflowEquipment[equipment->mRID];					
					if (pfEquipment->mTerminals.size() < term->sequenceNumber.value) {
						pfEquipment->mTerminals.resize(term->sequenceNumber.value);
					}
					pfEquipment->mTerminals[term->sequenceNumber-1] = mPowerflowTerminals[term->mRID];

					mLog.Log(Logger::Level::INFO) << "        " << "Equipment " << equipment->mRID
						<< ", sequenceNumber " << term->sequenceNumber-1 << std::endl;
					mLog.Log(Logger::Level::DEBUG) << "    " << "Added node " << nodesVec[term->sequenceNumber - 1].mSimNode << " to equipment" << std::endl;
				}
			}
		}
	}
	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mVoltages = new SvVoltage*[mPowerflowNodes.size()];
	mLog.Log(Logger::Level::INFO) << "#### List of node voltages from power flow calculation ####" << std::endl;

	for (auto obj : mModel.Objects) {
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			TopologicalNode* node = volt->TopologicalNode;
			if (!node) {
				mLog.Log(Logger::Level::WARN) << "SvVoltage references missing Topological Node, ignoring" << std::endl;
				continue;
			}
			auto search = mPowerflowNodes.find(node->mRID);
			if (search == mPowerflowNodes.end()) {
				mLog.Log(Logger::Level::WARN) << "SvVoltage references Topological Node " << node->mRID
					<< " missing from mTopNodes, ignoring" << std::endl;
				continue;
			}
			mPowerflowNodes[node->mRID].mVoltageAbs = volt->v.value;
			mPowerflowNodes[node->mRID].mVoltagePhase = volt->angle.value;
			mLog.Log(Logger::Level::INFO) << "Node " << mPowerflowNodes[node->mRID].mSimNode << ": " << mPowerflowNodes[node->mRID].mVoltageAbs << "<"
				<< mPowerflowNodes[node->mRID].mVoltagePhase << std::endl;
		}
		else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			// TODO could there be more than one power flow per equipment?
			Terminal* term = flow->Terminal;
			ConductingEquipment* eq = term->ConductingEquipment;			

			mLog.Log(Logger::Level::INFO) << "    " << term->mRID << ": equipment " << eq->mRID << ", sequenceNumber "
				<< term->sequenceNumber << std::endl;
			std::vector<PowerflowNode> &nodesVec = mEqNodeMap[eq->mRID];
			if (nodesVec.size() < (unsigned)term->sequenceNumber) {
				nodesVec.resize(term->sequenceNumber);
			}
			nodesVec[term->sequenceNumber - 1] = mPowerflowNodes[topNode->mRID];
			mLog.Log(Logger::Level::DEBUG) << "    " << "Added node " << nodesVec[term->sequenceNumber - 1].mSimNode << " to equipment" << std::endl;

			mPowerFlows[eq->mRID] = PowerflowTerminal(flow->p.value, flow->q.value);
		}
	}

	mLog.Log(Logger::Level::INFO) << "#### Create new components ####" << std::endl;
	for (auto obj : mModel.Objects) {
		Component::Ptr comp = mapComponent(obj);
		if (comp)
			mComponents.push_back(comp);
	}
}

Component::List& Reader::getComponents() {
	return mComponents;
}

Matrix::Index Reader::mapTopologicalNode(String mrid) {
	auto search = mPowerflowNodes.find(mrid);
	if (search == mPowerflowNodes.end()) {
		return -1;
	}

	return search->second.mSimNode;
}

Component::Ptr Reader::mapEnergyConsumer(EnergyConsumer* consumer) {
	String mRID = consumer->mRID;
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(mRID);
	SvPowerFlow* flow;
	if (nodes.size() != 1) {
		mLog.Log(Logger::Level::WARN) << mRID << " has " << nodes.size() << " terminals; ignoring" << std::endl;
		return nullptr;
	}
	auto search = mPowerFlows.find(mRID);
	if (search == mPowerFlows.end()) {
		mLog.Log(Logger::Level::WARN) << mRID << " has no associated SvPowerFlow, ignoring" << std::endl;
		return nullptr;
	}
	flow = search->second;
	
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node];
	if (!volt) {
		mLog.Log(Logger::Level::WARN) << consumer->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}

	mLog.Log(Logger::Level::INFO) << "Found EnergyConsumer " << consumer->name << " rid=" << consumer->mRID << " node="
		<< node << " P=" << flow->p.value << " Q=" << flow->q.value
		<< " V=" << volt->v.value << "<" << volt->angle.value << std::endl;

	// Apply unit multipliers according to CGMES convetions.
	flow->p.value = Reader::unitValue(flow->p.value, UnitMultiplier::M);
	flow->q.value = Reader::unitValue(flow->q.value, UnitMultiplier::M);
	volt->v.value = Reader::unitValue(volt->v.value, UnitMultiplier::k);

	mLog.Log(Logger::Level::INFO) << "Create PQLoad " << consumer->name << " node="
		<< node << " P=" << flow->p.value << " Q=" << flow->q.value
		<< " V=" << volt->v.value << "<" << volt->angle.value << std::endl;
	return std::make_shared<Components::DP::PQLoad>(consumer->name, node, flow->p.value, flow->q.value, volt->v.value, volt->angle.value*PI / 180);
}

Component::Ptr Reader::mapACLineSegment(ACLineSegment* line) {
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(line->mRID);
	if (nodes.size() != 2) {
		mLog.Log(Logger::Level::WARN) << "ACLineSegment " << line->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		// TODO better error handling (throw exception?)
		return nullptr;
	}
	Int node1 = nodes[0];
	Int node2 = nodes[1];
	Real resistance = line->r.value;
	Real inductance = line->x.value / mFrequency;

	mLog.Log(Logger::Level::INFO) << "Found ACLineSegment " << line->name << " rid=" << line->mRID << " node1=" << node1 << " node2=" << node2
		<< " R=" << line->r.value << " X=" << line->x.value << std::endl;

	mLog.Log(Logger::Level::INFO) << "Create RxLine " << line->name << " node1=" << node1 << " node2=" << node2
		<< " R=" << resistance << " L=" << inductance << std::endl;
	return std::make_shared<Components::DP::RxLine>(line->name, node1, node2, resistance, inductance);
}


Component::Ptr Reader::mapExternalNetworkInjection(ExternalNetworkInjection* inj) {
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(inj->mRID);
	if (nodes.size() != 1) {
		mLog.Log(Logger::Level::ERROR) << "ExternalNetworkInjection " << inj->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node];
	if (!volt) {
		mLog.Log(Logger::Level::ERROR) << "ExternalNetworkInjection " << inj->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}
	Real voltAbs = unitValue(volt->v.value, UnitMultiplier::k);
	Real voltPhase = volt->angle.value;
	Complex initVoltage = std::polar(voltAbs, voltPhase * PI / 180);
	mLog.Log(Logger::Level::INFO) << "IdealVoltageSource " << inj->name << " rid=" << inj->mRID << " node1=" << node
		<< " V=" << voltAbs << "<" << voltPhase << std::endl;

	return std::make_shared<Components::DP::VoltageSource>(inj->name, node, GND, initVoltage);
}

Component::Ptr Reader::mapPowerTransformer(PowerTransformer* trans) {
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(trans->mRID);
	if (nodes.size() != trans->PowerTransformerEnd.size()) {
		mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has differing number of terminals and windings, ignoring" << std::endl;
		return nullptr;
	}
	if (nodes.size() != 2) {
		// TODO three windings also possible
		mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has " << nodes.size() << "terminals; ignoring" << std::endl;
		return nullptr;
	}

	mLog.Log(Logger::Level::INFO) << "Found PowerTransformer " << trans->name << " rid=" << trans->mRID
		<< " node1=" << nodes[0] << " node2=" << nodes[1] << std::endl;

	Int node1 = nodes[0];
	Int node2 = nodes[1];
	Real voltageNode1 = 0;
	Real inductanceNode1 = 0;
	Real resistanceNode1 = 0;
	Real voltageNode2 = 0;
	Real inductanceNode2 = 0;
	Real resistanceNode2 = 0;

	for (auto end : trans->PowerTransformerEnd) {
		if (end->endNumber == 1) {
			mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_1 " << end->name
				<< " Vrated=" << end->ratedU.value << " R=" << end->r.value << " X=" << end->x.value << std::endl;
			voltageNode1 = unitValue(end->ratedU.value, UnitMultiplier::k);
			inductanceNode1 = end->x.value / mFrequency;
			resistanceNode1 = end->r.value;
		}
		if (end->endNumber == 2) {
			mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_2 " << end->name
				<< " Vrated=" << end->ratedU.value << " R=" << end->r.value << " X=" << end->x.value << std::endl;
			voltageNode2 = unitValue(end->ratedU.value, UnitMultiplier::k);
		}
	}

	if (voltageNode1 != 0 && voltageNode2 != 0) {
		Real ratioAbs = voltageNode1 / voltageNode2;
		Real ratioPhase = 0;
		mLog.Log(Logger::Level::INFO) << "Create PowerTransformer " << trans->name
			<< " node1=" << node1 << " node2=" << node2
			<< " ratio=" << ratioAbs << "<" << ratioPhase
			<< " inductance=" << inductanceNode1 << std::endl;

		//return std::make_shared<TransformerDP>(trans->name, node1, node2, ratioAbs, ratioPhase, 0, inductanceNode1);
		return std::make_shared<Components::DP::TransformerIdeal>(trans->name, node1, node2, ratioAbs, ratioPhase);

	}
	mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has no primary End; ignoring" << std::endl;
	return nullptr;
}

// TODO: don't use SvVoltage, but map to a SynchronGeneratorDP instead
Component::Ptr Reader::mapSynchronousMachine(SynchronousMachine* machine)
{
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(machine->mRID);
	if (nodes.size() != 1) {
		// TODO: check with the model if this assumption (only 1 terminal) is always true
		mLog.Log(Logger::Level::WARN) << "SynchronousMachine " << machine->mRID << " has " << nodes.size()
			<< " terminals, ignoring" << std::endl;
		return nullptr;
	}
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node];
	if (!volt) {
		mLog.Log(Logger::Level::WARN) << "SynchronousMachine " << machine->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}

	// Apply unit multipliers according to CGMES convetions.
	Real voltAbs = unitValue(volt->v.value, UnitMultiplier::k);
	Real voltPhase = volt->angle.value * PI / 180;
	Complex initVoltage = std::polar(voltAbs, voltPhase);

	// TODO is it appropiate to use this resistance here
	mLog.Log(Logger::Level::INFO) << "Create IdealVoltageSource " << machine->name << " node=" << node
		<< " V=" << voltAbs << "<" << voltPhase << std::endl;
	return std::make_shared<Components::DP::VoltageSource>(machine->name, GND, node, initVoltage);
}

void Reader::mapAsynchronousMachine(AsynchronousMachine* machine)
{
}

void Reader::mapEquivalentInjection(EquivalentInjection* inj)
{
}
