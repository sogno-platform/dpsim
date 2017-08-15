#include "CIMReader.h"
#include "CIMModel.hpp"
#include "IEC61970.hpp"

#include "Components/RxLine.h"

using namespace DPsim;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Core;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;

// TODO is UnitMulitplier actually used/set anywhere?
double CIMReader::unitValue(double value, UnitMultiplier mult) {
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

CIMReader::CIMReader() {
	mModel.setDependencyCheckOff();
}

CIMReader::~CIMReader() {
}

BaseComponent* CIMReader::mapACLineSegment(ACLineSegment* line) {
	std::vector<int> &nodes = mEqNodeMap.at(line->mRID); // TODO can fail
	if (nodes.size() != 2) {
		std::cerr << "ACLineSegment " << line->mRID << " has " << nodes.size() << " terminals, ignoring" << std::endl;
		// TODO better error handling (throw exception?)
		return nullptr;
	}
	Real r = line->r.value;
	Real x = line->x.value;
	std::cerr << "RxLine " << line->name << " rid=" << line->mRID << " node1=" << nodes[0] << " node2=" << nodes[1];
	std::cerr << " R=" << r << " X=" << x << std::endl;
	return new RxLine(line->name, nodes[0], nodes[1], r, x);
}

BaseComponent* CIMReader::mapComponent(BaseClass* obj) {
	// TODO can we do this nicer than with a giant dynamic switch?
	ACLineSegment* line;
	if ((line = dynamic_cast<ACLineSegment*>(obj)))
		return mapACLineSegment(line);
	return nullptr;
}

bool CIMReader::addFile(std::string filename) {
	return mModel.addCIMFile(filename);
}

std::vector<BaseComponent*> CIMReader::mapComponents() {
	std::vector<BaseComponent*> compVector;

	mModel.parseFiles();
	// First, go through all topological nodes and collect them in a list.
	// Since all nodes have references to the equipment connected to them (via Terminals), but not
	// the other way around (which we need for instantiating the components), we collect that information here as well.
	for (BaseClass* obj : mModel.Objects) {
		TopologicalNode* topNode = dynamic_cast<TopologicalNode*>(obj);
		if (topNode) {
			std::cerr << "TopologicalNode " << mTopNodes.size()+1 << " rid=" << topNode->mRID << " Terminals:" << std::endl;
			mTopNodes.push_back(topNode);
			for (Terminal* term : topNode->Terminal) {
				std::cerr << "    " << term->mRID << std::endl;
				ConductingEquipment *eq = term->ConductingEquipment;
				if (!eq) {
					std::cerr << "Terminal " << term->mRID << " has no Conducting Equipment, ignoring!" << std::endl;
				} else {
					std::cerr << "    eq " << eq->mRID << " sequenceNumber " << term->sequenceNumber << std::endl;
					std::vector<int> &nodesVec = mEqNodeMap[eq->mRID];
					if (nodesVec.size() < term->sequenceNumber)
						nodesVec.resize(term->sequenceNumber);
					nodesVec[term->sequenceNumber-1] = mTopNodes.size();
				}
			}
		}
	}
	for (BaseClass* obj : mModel.Objects) {
		BaseComponent* comp = mapComponent(obj);
		if (comp)
			compVector.push_back(comp);
	}
	return compVector;
}
