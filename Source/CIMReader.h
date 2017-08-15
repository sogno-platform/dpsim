#ifndef CIMREADER_H
#define CIMREADER_H

#include <vector>

#include "Components/BaseComponent.h"

#include "CIMModel.hpp"
#include "IEC61970.hpp"

using namespace DPsim;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;

namespace DPsim {
	class CIMReader {
		private:
			CIMModel mModel;
			// List of topological nodes. Offset by 1 to the index used for the
			// component constructors (first entry in this vector should correspond
			// to giving a 1 in the component constructor).
			//
			// TODO do we even need this?
			std::vector<TopologicalNode*> mTopNodes;
			// Maps the RID of a ConductingEquipment to a list of nodes as given in
			// the component constructors.
			std::map<std::string, std::vector<int>> mEqNodeMap;

			BaseComponent* mapComponent(BaseClass* obj);
			BaseComponent* mapACLineSegment(ACLineSegment* line);
		public:
			CIMReader();
			virtual ~CIMReader();

			bool addFile(std::string filename);
			std::vector<BaseComponent*> mapComponents();

			static double unitValue(double value, UnitMultiplier mult);
	};
};

#endif
