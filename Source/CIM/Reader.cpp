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

// TODO is UnitMulitplier actually used/set anywhere?
Real Reader::unitValue(Real value, UnitMultiplier mult)
{
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
Reader::Reader(Real systemFrequency, Logger& logger)
{
	mModel.setDependencyCheckOff();
	mNumVoltageSources = 0;
	mVoltages = nullptr;
	mFrequency = systemFrequency;
	mLogger = &logger;
}

Reader::~Reader()
{
	if (mVoltages)
		delete[] mVoltages;
}

Components::Base::Ptr Reader::mapACLineSegment(ACLineSegment* line)
{
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(line->mRID); // TODO can fail
	if (nodes.size() != 2) {
		mLogger->Log(Logger::Level::WARN) << "ACLineSegment " << line->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		// TODO better error handling (throw exception?)
		return nullptr;
	}
	Int node1 = nodes[0];
	Int node2 = nodes[1];
	Real resistance = line->r.value;
	Real inductance = line->x.value/mFrequency;

	mLogger->Log(Logger::Level::INFO) << "Found ACLineSegment " << line->name << " rid=" << line->mRID << " node1=" << node1 << " node2=" << node2
		<< " R=" << line->r.value << " X=" << line->x.value << std::endl;

	mLogger->Log(Logger::Level::INFO) << "Create RxLine " << line->name << " node1=" << node1 << " node2=" << node2
		<< " R=" << resistance << " L=" << inductance << std::endl;
	return std::make_shared<Components::DP::RxLine>(line->name, node1, node2, resistance, inductance);
}

void Reader::mapAsynchronousMachine(AsynchronousMachine* machine)
{
}

void Reader::mapEnergyConsumer(EnergyConsumer* con)
{
}

void Reader::mapEquivalentInjection(EquivalentInjection* inj)
{
}

Components::Base::Ptr Reader::mapExternalNetworkInjection(ExternalNetworkInjection* inj)
{
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(inj->mRID);
	if (nodes.size() != 1) {
		mLogger->Log(Logger::Level::ERROR) << "ExternalNetworkInjection " << inj->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		mLogger->Log(Logger::Level::ERROR) << "ExternalNetworkInjection " << inj->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}
	Real voltAbs = unitValue(volt->v.value, UnitMultiplier::k);
	Real voltPhase = volt->angle.value;
	Complex initVoltage = std::polar(voltAbs, voltPhase * PI / 180);
	mLogger->Log(Logger::Level::INFO) << "IdealVoltageSource " << inj->name << " rid=" << inj->mRID << " node1=" << node
		<< " V=" << voltAbs << "<" << voltPhase << std::endl;

	return std::make_shared<Components::DP::VoltageSource>(inj->name, node, 0, initVoltage);
}

// TODO: support phase shift
Components::Base::Ptr Reader::mapPowerTransformer(PowerTransformer* trans) {
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(trans->mRID);
	if (nodes.size() != trans->PowerTransformerEnd.size()) {
		mLogger->Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has differing number of terminals and windings, ignoring" << std::endl;
		return nullptr;
	}
	if (nodes.size() != 2) {
		// TODO three windings also possible
		mLogger->Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has " << nodes.size() << "terminals; ignoring" << std::endl;
		return nullptr;
	}

	mLogger->Log(Logger::Level::INFO) << "Found PowerTransformer " << trans->name << " rid=" << trans->mRID
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
			mLogger->Log(Logger::Level::INFO) << "    PowerTransformerEnd_1 " << end->name
				<< " Vrated=" << end->ratedU.value << " R=" << end->r.value << " X=" << end->x.value << std::endl;
			voltageNode1 = unitValue(end->ratedU.value, UnitMultiplier::k);
			inductanceNode1 = end->x.value / mFrequency;
			resistanceNode1 = end->r.value;
		}
		if (end->endNumber == 2) {
			mLogger->Log(Logger::Level::INFO) << "    PowerTransformerEnd_2 " << end->name
				<< " Vrated=" << end->ratedU.value << " R=" << end->r.value << " X=" << end->x.value << std::endl;
			voltageNode2 = unitValue(end->ratedU.value, UnitMultiplier::k);
		}
	}

	if (voltageNode1 != 0 && voltageNode2 != 0) {
		Real ratioAbs = voltageNode1 / voltageNode2;
		Real ratioPhase = 0;
		mLogger->Log(Logger::Level::INFO) << "Create PowerTransformer " << trans->name
			<< " node1=" << node1 << " node2=" << node2
			<< " ratio=" << ratioAbs << "<" << ratioPhase
			<< " inductance=" << inductanceNode1 << std::endl;

		//return std::make_shared<TransformerDP>(trans->name, node1, node2, ratioAbs, ratioPhase, 0, inductanceNode1);
		return std::make_shared<Components::DP::TransformerIdeal>(trans->name, node1, node2, ratioAbs, ratioPhase);

	}
	mLogger->Log(Logger::Level::WARN) << "PowerTransformer " << trans->mRID << " has no primary End; ignoring" << std::endl;
	return nullptr;
}

// TODO: don't use SvVoltage, but map to a SynchronGeneratorDP instead
Components::Base::Ptr Reader::mapSynchronousMachine(SynchronousMachine* machine)
{
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(machine->mRID);
	if (nodes.size() != 1) {
		// TODO: check with the model if this assumption (only 1 terminal) is always true
		mLogger->Log(Logger::Level::WARN) << "SynchronousMachine " << machine->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		return nullptr;
	}
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		mLogger->Log(Logger::Level::WARN) << "SynchronousMachine " << machine->mRID << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}

	// Apply unit multipliers according to CGMES convetions.
	Real voltAbs = unitValue(volt->v.value, UnitMultiplier::k);
	Real voltPhase = volt->angle.value * PI / 180;
	Complex initVoltage = std::polar(voltAbs, voltPhase);

	// TODO is it appropiate to use this resistance here
	mLogger->Log(Logger::Level::INFO) << "Create IdealVoltageSource " << machine->name << " node=" << node
		<< " V=" << voltAbs << "<" << voltPhase << std::endl;
	return std::make_shared<Components::DP::VoltageSource>(machine->name, node, 0, initVoltage);
}

Components::Base::Ptr Reader::mapComponent(BaseClass* obj)
{
	if (ACLineSegment *line = dynamic_cast<ACLineSegment*>(obj))
		return mapACLineSegment(line);
	if (ExternalNetworkInjection *inj = dynamic_cast<ExternalNetworkInjection*>(obj))
		return mapExternalNetworkInjection(inj);
	if (PowerTransformer *trans = dynamic_cast<PowerTransformer*>(obj))
		return mapPowerTransformer(trans);
	if (SynchronousMachine *syncMachine = dynamic_cast<SynchronousMachine*>(obj))
		return mapSynchronousMachine(syncMachine);
	return nullptr;
}

Components::Base::Ptr Reader::newPQLoad(String rid, String name)
{
	std::vector<Matrix::Index> &nodes = mEqNodeMap.at(rid);
	if (nodes.size() != 1) {
		mLogger->Log(Logger::Level::WARN) << rid << " has " << nodes.size() << " terminals; ignoring" << std::endl;
		return nullptr;
	}
	auto search = mPowerFlows.find(rid);
	if (search == mPowerFlows.end()) {
		mLogger->Log(Logger::Level::WARN) << rid << " has no associated SvPowerFlow, ignoring" << std::endl;
		return nullptr;
	}
	SvPowerFlow* flow = search->second;
	Int node = nodes[0];
	SvVoltage *volt = mVoltages[node-1];
	if (!volt) {
		mLogger->Log(Logger::Level::WARN) << rid << " has no associated SvVoltage, ignoring" << std::endl;
		return nullptr;
	}

	mLogger->Log(Logger::Level::INFO) << "Found EnergyConsumer " << name << " rid=" << rid << " node="
		<< node << " P=" << flow->p.value << " Q=" << flow->q.value
		<< " V=" << volt->v.value << "<" << volt->angle.value << std::endl;

	// Apply unit multipliers according to CGMES convetions.
	flow->p.value = Reader::unitValue(flow->p.value, UnitMultiplier::M);
	flow->q.value = Reader::unitValue(flow->q.value, UnitMultiplier::M);
	volt->v.value = Reader::unitValue(volt->v.value, UnitMultiplier::k);

	mLogger->Log(Logger::Level::INFO) << "Create PQLoad " << name << " node="
		<< node << " P=" << flow->p.value << " Q=" << flow->q.value
		<< " V=" << volt->v.value << "<" << volt->angle.value << std::endl;
	return std::make_shared<Components::DP::PQLoad>(name, node, flow->p.value, flow->q.value, volt->v.value, volt->angle.value*PI/180);
}

bool Reader::addFile(String filename)
{
	return mModel.addCIMFile(filename);
}

void Reader::parseFiles()
{
	mModel.parseFiles();
	// First, go through all topological nodes and collect them in a list.
	// Since all nodes have references to the equipment connected to them (via Terminals), but not
	// the other way around (which we need for instantiating the components), we collect that information here as well.
	mLogger->Log(Logger::Level::INFO) << "#### List of topological nodes and associated terminals ####" << std::endl;

	for (auto obj : mModel.Objects) {
		TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj);
		if (topNode) {
			mLogger->Log(Logger::Level::INFO) << "TopologicalNode " << mTopNodes.size()+1 << " rid=" << topNode->mRID << " Terminals:" << std::endl;
			mTopNodes[topNode->mRID] = (Matrix::Index) mTopNodes.size()+1;

			for (auto term : topNode->Terminal) {
				mLogger->Log(Logger::Level::INFO) << "    " << term->mRID << std::endl;
				ConductingEquipment *eq = term->ConductingEquipment;
				if (!eq) {
					mLogger->Log(Logger::Level::WARN) << "Terminal " << term->mRID << " has no Conducting Equipment, ignoring!" << std::endl;
				}
				else {
					mLogger->Log(Logger::Level::INFO) << "    eq " << eq->mRID << " sequenceNumber " << term->sequenceNumber << std::endl;
					std::vector<Matrix::Index> &nodesVec = mEqNodeMap[eq->mRID];
					if (nodesVec.size() < (unsigned) term->sequenceNumber) {
						nodesVec.resize(term->sequenceNumber);
					}
					nodesVec[term->sequenceNumber-1] = (Matrix::Index) mTopNodes.size();
				}
			}
		}
	}
	// Collect voltage state variables associated to nodes that are used
	// for various components.
	mVoltages = new SvVoltage*[mTopNodes.size()];
	mLogger->Log(Logger::Level::INFO) << "#### List of node voltages from power flow calculation ####" << std::endl;

	for (auto obj : mModel.Objects) {
		if (SvVoltage* volt = dynamic_cast<SvVoltage*>(obj)) {
			TopologicalNode* node = volt->TopologicalNode;
			if (!node) {
				mLogger->Log(Logger::Level::WARN) << "SvVoltage references missing Topological Node, ignoring" << std::endl;
				continue;
			}
			auto search = mTopNodes.find(node->mRID);
			if (search == mTopNodes.end()) {
				mLogger->Log(Logger::Level::WARN) << "SvVoltage references Topological Node " << node->mRID << " missing from mTopNodes, ignoring" << std::endl;
				continue;
			}
			mVoltages[search->second-1] = volt;
			mLogger->Log(Logger::Level::INFO) << "Node " << search->second << ": " << volt->v.value << "<" << volt->angle.value << std::endl;
		}
		else if (SvPowerFlow* flow = dynamic_cast<SvPowerFlow*>(obj)) {
			// TODO could there be more than one power flow per equipment?
			Terminal* term = flow->Terminal;
			mPowerFlows[term->ConductingEquipment->mRID] = flow;
		}
	}

	mLogger->Log(Logger::Level::INFO) << "#### Create new components ####" << std::endl;
	for (auto obj : mModel.Objects) {
		Components::Base::Ptr comp = mapComponent(obj);
		if (comp)
			mComponents.push_back(comp);
	}
}

Components::Base::List& Reader::getComponents()
{
	return mComponents;
}

Matrix::Index Reader::mapTopologicalNode(String mrid)
{
	auto search = mTopNodes.find(mrid);
	if (search == mTopNodes.end())
		return -1;

	return search->second;
}

int Reader::getNumVoltageSources()
{
	return mNumVoltageSources;
}
