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

Reader::Reader(Real systemFrequency, Logger::Level logLevel, Logger::Level componentLogLevel)
	: mLog("Logs/CIMpp.log", logLevel) {
	mModel.setDependencyCheckOff();
	mFrequency = systemFrequency;
	mOmega = 2 * PI*mFrequency;
	mComponentLogLevel = componentLogLevel;
}

Reader::~Reader() { }

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
	mLog.Log(Logger::Level::INFO) << "#### List of TopologicalNodes, associated Terminals and Equipment" << std::endl;

	for (auto obj : mModel.Objects) {
		if (TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj)) {
			processTopologicalNode(topNode);
		}
	}
	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mLog.Log(Logger::Level::INFO) << "#### List of Node voltages and Terminal power flow data" << std::endl;

	for (auto obj : mModel.Objects) {
		// Check if object is of class SvVoltage
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			processSvVoltage(volt);
		}
		// Check if object is of class SvPowerFlow
		else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			processSvPowerFlow(flow);
		}
	}

	mLog.Log(Logger::Level::INFO) << "#### Create other components" << std::endl;
	for (auto obj : mModel.Objects) {
		// Check if object is not TopologicalNode, SvVoltage or SvPowerFlow
		if (!dynamic_cast<TopologicalNode*>(obj)
			&& !dynamic_cast<SvVoltage*>(obj)
			&& !dynamic_cast<SvPowerFlow*>(obj)) {
			if (IdentifiedObject* idObj = dynamic_cast<IdentifiedObject*>(obj)) {
			// Check if object is already in equipment list
				if (mPowerflowEquipment.find(idObj->mRID) == mPowerflowEquipment.end()) {
					Component::Ptr comp = mapComponent(obj);
					if (comp) mPowerflowEquipment.insert(std::make_pair(comp->mUID, comp));
				}
			}
		}
	}
}

void Reader::processTopologicalNode(TopologicalNode* topNode) {
	// Add this node to global node list and assign simulation node incrementally.
	mPowerflowNodes[topNode->mRID] = std::make_shared<Node>(topNode->mRID, (Matrix::Index) mPowerflowNodes.size());

	mLog.Log(Logger::Level::INFO) << "TopologicalNode " << topNode->mRID
		<< " as simulation node " << mPowerflowNodes[topNode->mRID]->mSimNode << std::endl;

	for (auto term : topNode->Terminal) {
		// Insert Terminal if it does not exist in the map and add reference to node.
		// This could be optimized because the Terminal is searched twice.
		mPowerflowTerminals.insert(std::make_pair(term->mRID, std::make_shared<Terminal>(term->mRID)));
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
			if (mPowerflowEquipment.find(equipment->mRID) == mPowerflowEquipment.end()) {
				Component::Ptr comp = mapComponent(equipment);
				if (comp) mPowerflowEquipment.insert(std::make_pair(equipment->mRID, comp));
			}				
			auto pfEquipment = mPowerflowEquipment[equipment->mRID];			
			pfEquipment->setTerminalAt(mPowerflowTerminals[term->mRID], term->sequenceNumber-1);

			mLog.Log(Logger::Level::INFO) << "Added Terminal "
				<< term->mRID << " to Equipment " << equipment->mRID << std::endl;
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
	Real voltageAbs = Reader::unitValue(volt->v.value, UnitMultiplier::k);
	Real voltagePhase = volt->angle.value * PI / 180;
	mPowerflowNodes[node->mRID]->mVoltage = std::polar<Real>(voltageAbs, voltagePhase);
	mLog.Log(Logger::Level::INFO) << "Node " << mPowerflowNodes[node->mRID]->mUID
		<< " SimNode " << mPowerflowNodes[node->mRID]->mSimNode << ": "
		<< std::abs(mPowerflowNodes[node->mRID]->mVoltage) << " V, "
		<< std::arg(mPowerflowNodes[node->mRID]->mVoltage)*180/PI << " deg" << std::endl;
}

void Reader::processSvPowerFlow(SvPowerFlow* flow) {
	IEC61970::Base::Core::Terminal* term = flow->Terminal;
	ConductingEquipment* eq = term->ConductingEquipment;

	mPowerflowTerminals[term->mRID]->mPower = { Reader::unitValue(flow->p.value, UnitMultiplier::M), Reader::unitValue(flow->q.value, UnitMultiplier::M) };

	mLog.Log(Logger::Level::INFO) << "Terminal " << term->mRID << ": "
		<< mPowerflowTerminals[term->mRID]->mPower.real() << " W + j"
		<< mPowerflowTerminals[term->mRID]->mPower.imag() << " Var" << std::endl;
}

Component::List Reader::getComponents() {
	Component::List components;
	for (auto comp : mPowerflowEquipment) {
		components.push_back(comp.second);
	}
	return components;
}

Node::List Reader::getNodes() {
	Node::List nodes;
	nodes.resize(mPowerflowNodes.size());	
	for (auto node : mPowerflowNodes) {
		nodes[node.second->mSimNode] = node.second;
	}
	return nodes;
}

Matrix::Index Reader::mapTopologicalNode(String mrid) {
	auto search = mPowerflowNodes.find(mrid);
	if (search == mPowerflowNodes.end()) {
		return -1;
	}
	return search->second->mSimNode;
}

Component::Ptr Reader::mapEnergyConsumer(EnergyConsumer* consumer) {
	mLog.Log(Logger::Level::INFO) << "Found EnergyConsumer " << consumer->name << std::endl;	
	return std::make_shared<Components::DP::PQLoad>(consumer->mRID, consumer->name, mComponentLogLevel);
}

Component::Ptr Reader::mapACLineSegment(ACLineSegment* line) {	
	mLog.Log(Logger::Level::INFO) << "Found ACLineSegment " << line->name
		<< " r=" << line->r.value << " x=" << line->x.value
		<< " length=" << line->length.value << std::endl;

	Real resistance = line->r.value;
	Real inductance = line->x.value / mOmega;
	if (line->length.value > 0) {
		resistance = line->r.value * line->length.value;
		inductance = line->x.value / mOmega * line->length.value;
	}

	return std::make_shared<Components::DP::RxLine>(line->mRID, line->name, resistance, inductance, mComponentLogLevel);
}

Component::Ptr Reader::mapPowerTransformer(PowerTransformer* trans) {
	if (trans->PowerTransformerEnd.size() != 2) {
		mLog.Log(Logger::Level::WARN) << "PowerTransformer " << trans->name
			<< " does not have exactly two windings, ignoring" << std::endl;
		return nullptr;
	}	
	mLog.Log(Logger::Level::INFO) << "Found PowerTransformer " << trans->name << std::endl;

	PowerTransformerEnd* end1;
	PowerTransformerEnd* end2;
	for (auto end : trans->PowerTransformerEnd) {
		if (end->Terminal->sequenceNumber == 1) end1 = end;
		else if (end->Terminal->sequenceNumber == 2) end2 = end;
		else return nullptr;
	}

	mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_1 " << end1->name
		<< " Vrated=" << end1->ratedU.value << " R=" << end1->r.value << " X=" << end1->x.value << std::endl;	
	mLog.Log(Logger::Level::INFO) << "    PowerTransformerEnd_2 " << end2->name
		<< " Vrated=" << end2->ratedU.value << " R=" << end2->r.value << " X=" << end2->x.value << std::endl;

	Real voltageNode1 = unitValue(end1->ratedU.value, UnitMultiplier::k);
	Real voltageNode2 = unitValue(end2->ratedU.value, UnitMultiplier::k);
	Real ratioAbs = voltageNode1 / voltageNode2;
	Real ratioPhase = 0;

	Real inductance = 0;
	Real resistance = 0;
	if (voltageNode1 > voltageNode2 && end1->x.value > 0.001) {
		inductance = end1->x.value / mOmega;
		resistance = end1->r.value;
	} else if (voltageNode1 > voltageNode2 && end2->x.value > 0.001) {
		inductance = end2->x.value / mOmega / std::pow(ratioAbs, 2);
		resistance = end2->r.value / std::pow(ratioAbs, 2);
	}
	else if (voltageNode2 > voltageNode1 && end2->x.value > 0.001) {
		inductance = end2->x.value / mOmega;
		resistance = end2->r.value;
	}
	else if (voltageNode2 > voltageNode1 && end1->x.value > 0.001) {
		inductance = end1->x.value / mOmega / std::pow(ratioAbs, 2);
		resistance = end1->r.value / std::pow(ratioAbs, 2);
	}
		
	return std::make_shared<Components::DP::Transformer>(trans->mRID, trans->name, ratioAbs, ratioPhase, 0, inductance, mComponentLogLevel);
}

Component::Ptr Reader::mapSynchronousMachine(SynchronousMachine* machine) {	
	mLog.Log(Logger::Level::INFO) << "Found  Synchronous machine " << machine->name << std::endl;
	return std::make_shared<Components::DP::SynchronGeneratorIdeal>(machine->mRID, machine->name, mComponentLogLevel);
}
