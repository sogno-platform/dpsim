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


Reader::Reader(Real systemFrequency, Logger::Level logLevel) : mLog("Logs/CIMpp.log", logLevel) {
	mModel.setDependencyCheckOff();
	mFrequency = systemFrequency;
}

Reader::~Reader() {

}

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
		if (TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj)) {
			processTopologicalNode(topNode);
		}
	}
	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mLog.Log(Logger::Level::INFO) << "#### List of node voltages from power flow calculation ####" << std::endl;

	for (auto obj : mModel.Objects) {
		// Check if object is of class SvVoltage
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			processSvVoltage(volt);
		}
		// Check if object is of class SvPowerFlow
		else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			Terminal* term = flow->Terminal;
			ConductingEquipment* eq = term->ConductingEquipment;			

			mPowerflowTerminals[term->mRID]->mActivePower = flow->p.value;
			mPowerflowTerminals[term->mRID]->mReactivePower = flow->q.value;

			mLog.Log(Logger::Level::INFO) << "Terminal " << term->mRID << ":"
				<< mPowerflowTerminals[term->mRID]->mActivePower << " + j"
				<< mPowerflowTerminals[term->mRID]->mReactivePower << std::endl;
		}
	}

	mLog.Log(Logger::Level::INFO) << "#### Create new components ####" << std::endl;
	for (auto obj : mModel.Objects) {
		Component::Ptr comp = mapComponent(obj);
		if (comp) {
			mComponents.push_back(comp);
		}
	}
}

void Reader::processTopologicalNode(TopologicalNode* topNode) {
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
			pfEquipment->mTerminals[term->sequenceNumber - 1] = mPowerflowTerminals[term->mRID];

			mLog.Log(Logger::Level::INFO) << "        " << "Equipment " << equipment->mRID
				<< ", sequenceNumber " << term->sequenceNumber - 1 << std::endl;
		}
	}
}

void Reader::processSvVoltage(SvVoltage* volt) {
	TopologicalNode* node = volt->TopologicalNode;
	if (!node) {
		mLog.Log(Logger::Level::WARN) << "SvVoltage references missing Topological Node, ignoring" << std::endl;
		return;
	}
	auto search = mPowerflowNodes.find(node->mRID);
	if (search == mPowerflowNodes.end()) {
		mLog.Log(Logger::Level::WARN) << "SvVoltage references Topological Node " << node->mRID
			<< " missing from mTopNodes, ignoring" << std::endl;
		return;
	}
	mPowerflowNodes[node->mRID]->mVoltageAbs = volt->v.value;
	mPowerflowNodes[node->mRID]->mVoltagePhase = volt->angle.value;
	mLog.Log(Logger::Level::INFO) << "Node " << mPowerflowNodes[node->mRID]->mSimNode << ": " << mPowerflowNodes[node->mRID]->mVoltageAbs << "<"
		<< mPowerflowNodes[node->mRID]->mVoltagePhase << std::endl;
}

void Reader::processSvPowerFlow(SvPowerFlow* flow) {

}

Component::List& Reader::getComponents() {
	return mComponents;
}

Matrix::Index Reader::mapTopologicalNode(String mrid) {
	auto search = mPowerflowNodes.find(mrid);
	if (search == mPowerflowNodes.end()) {
		return -1;
	}
	return search->second->mSimNode;
}

Component::Ptr Reader::mapEnergyConsumer(EnergyConsumer* consumer) {
	
	if (mPowerflowEquipment[consumer->mRID]->mTerminals.size() != 1) {
		mLog.Log(Logger::Level::WARN) << consumer->mRID << " has "
			<< mPowerflowEquipment[consumer->mRID]->mTerminals.size()
			<< " terminals; ignoring" << std::endl;
		return nullptr;
	}

	std::shared_ptr<PowerflowTerminal> term = mPowerflowEquipment[consumer->mRID]->mTerminals[0];
	std::shared_ptr<PowerflowNode> node = term->mNode;

	mLog.Log(Logger::Level::INFO) << "Found EnergyConsumer " << consumer->name
		<< " rid=" << consumer->mRID << " node=" << node->mSimNode
		<< " P=" << term->mActivePower << " Q=" << term->mReactivePower
		<< " V=" << node->mVoltageAbs << "<" << node->mVoltagePhase << std::endl;
	
	// Apply unit multipliers according to CGMES convetions.
	term->mActivePower = Reader::unitValue(term->mActivePower, UnitMultiplier::M);
	term->mReactivePower = Reader::unitValue(term->mReactivePower, UnitMultiplier::M);
	node->mVoltageAbs = Reader::unitValue(node->mVoltageAbs, UnitMultiplier::k);
	node->mVoltagePhase = node->mVoltagePhase * PI / 180;

	mLog.Log(Logger::Level::INFO) << "Create PQLoad " << consumer->name << " node=" << node->mSimNode
		<< " P=" << term->mActivePower << " Q=" << term->mReactivePower
		<< " V=" << node->mVoltageAbs << "<" << node->mVoltagePhase << std::endl;

	return std::make_shared<Components::DP::PQLoad>(consumer->name, node->mSimNode,
		term->mActivePower, term->mReactivePower, node->mVoltageAbs, node->mVoltagePhase);
}

Component::Ptr Reader::mapACLineSegment(ACLineSegment* line) {
	
	if (mPowerflowEquipment[line->mRID]->mTerminals.size() != 2) {
		mLog.Log(Logger::Level::WARN) << "ACLineSegment " << line->mRID << " has "
			<< mPowerflowEquipment[line->mRID]->mTerminals.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}

	std::shared_ptr<PowerflowNode> node1 = mPowerflowEquipment[line->mRID]->mTerminals[0]->mNode;
	std::shared_ptr<PowerflowNode> node2 = mPowerflowEquipment[line->mRID]->mTerminals[1]->mNode;
	Real resistance = line->r.value;
	Real inductance = line->x.value / mFrequency;

	mLog.Log(Logger::Level::INFO) << "Found ACLineSegment " << line->name
		<< " rid=" << line->mRID << " node1=" << node1->mSimNode << " node2=" << node2->mSimNode
		<< " R=" << line->r.value << " X=" << line->x.value << std::endl;

	mLog.Log(Logger::Level::INFO) << "Create RxLine " << line->name
		<< " node1=" << node1->mSimNode << " node2=" << node2->mSimNode
		<< " R=" << resistance << " L=" << inductance << std::endl;
	return std::make_shared<Components::DP::RxLine>(line->name, node1->mSimNode, node2->mSimNode, resistance, inductance);
}


Component::Ptr Reader::mapPowerTransformer(PowerTransformer* trans) {

	if (mPowerflowEquipment[trans->mRID]->mTerminals.size() != trans->PowerTransformerEnd.size()) {
		mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID
			<< " has differing number of terminals and windings, ignoring" << std::endl;
		return nullptr;
	}
	if (mPowerflowEquipment[trans->mRID]->mTerminals.size() != 2) {
		// TODO three windings also possible
		mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID
			<< " has " << mPowerflowEquipment[trans->mRID]->mTerminals.size() << "terminals; ignoring" << std::endl;
		return nullptr;
	}

	std::shared_ptr<PowerflowNode> node1 = mPowerflowEquipment[trans->mRID]->mTerminals[0]->mNode;
	std::shared_ptr<PowerflowNode> node2 = mPowerflowEquipment[trans->mRID]->mTerminals[1]->mNode;
	
	mLog.Log(Logger::Level::INFO) << "Found PowerTransformer " << trans->name << " rid=" << trans->mRID
		<< " node1=" << node1 << " node2=" << node2 << std::endl;

	PowerTransformerEnd* end1;
	PowerTransformerEnd* end2;

	for (auto end : trans->PowerTransformerEnd) {
		if (end->endNumber == 1) end1 = end;
		else if (end->endNumber == 2) end2 = end;
		else return nullptr;
	}

	mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_1 " << end1->name
		<< " Vrated=" << end1->ratedU.value << " R=" << end1->r.value << " X=" << end1->x.value << std::endl;	

	mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_2 " << end2->name
		<< " Vrated=" << end2->ratedU.value << " R=" << end2->r.value << " X=" << end2->x.value << std::endl;

	Real voltageNode1 = unitValue(end1->ratedU.value, UnitMultiplier::k);
	Real inductanceNode1 = end1->x.value / mFrequency;
	Real resistanceNode1 = end1->r.value;
	Real voltageNode2 = unitValue(end2->ratedU.value, UnitMultiplier::k);
	Real inductanceNode2 = end2->x.value / mFrequency;
	Real resistanceNode2 = end2->r.value;

	Real ratioAbs = voltageNode1 / voltageNode2;
	Real ratioPhase = 0;
	mLog.Log(Logger::Level::INFO) << "Create PowerTransformer " << trans->name
		<< " node1=" << node1->mSimNode << " node2=" << node2->mSimNode
		<< " ratio=" << ratioAbs << "<" << ratioPhase
		<< " inductance=" << inductanceNode1 << std::endl;

	//return std::make_shared<TransformerDP>(trans->name, node1, node2, ratioAbs, ratioPhase, 0, inductanceNode1);
	return std::make_shared<Components::DP::TransformerIdeal>(trans->name, node1->mSimNode, node2->mSimNode, ratioAbs, ratioPhase);
}


Component::Ptr Reader::mapSynchronousMachine(SynchronousMachine* machine) {

	if (mPowerflowEquipment[machine->mRID]->mTerminals.size() != 1) {
		mLog.Log(Logger::Level::WARN) << "SynchronousMachine " << machine->mRID << " has "
			<< mPowerflowEquipment[machine->mRID]->mTerminals.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}

	std::shared_ptr<PowerflowNode> node = mPowerflowEquipment[machine->mRID]->mTerminals[0]->mNode;

	// Apply unit multipliers according to CGMES convetions.
	Real voltAbs = unitValue(node->mVoltageAbs, UnitMultiplier::k);
	Real voltPhase = node->mVoltagePhase * PI / 180;
	Complex initVoltage = std::polar(voltAbs, voltPhase);

	mLog.Log(Logger::Level::INFO) << "Create IdealVoltageSource " << machine->name << " node=" << node->mSimNode
		<< " V=" << voltAbs << "<" << voltPhase << std::endl;
	return std::make_shared<Components::DP::VoltageSource>(machine->name, GND, node->mSimNode, initVoltage);
}

Component::Ptr Reader::mapExternalNetworkInjection(ExternalNetworkInjection* inj) {
	return nullptr;
}

