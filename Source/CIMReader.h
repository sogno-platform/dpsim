#ifndef CIMREADER_H
#define CIMREADER_H

#include <map>
#include <vector>

#include "Components/BaseComponent.h"

#include "CIMModel.hpp"
#include "IEC61970.hpp"

using namespace DPsim;
using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Equivalents;
using namespace IEC61970::Base::StateVariables;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;

namespace DPsim {
	class CIMReader {
		private:
			CIMModel mModel;
			// System frequency (has to be given to convert between reactances
			// in CIM and inductances used inside the simulation)
			Real mFrequency;
			// Maps the RID of a topological node to its simulation matrix index
			// as given in the component constructors (1 for the first node).
			std::map<std::string, int> mTopNodes;
			// Maps the RID of a ConductingEquipment to a list of nodes as given in
			// the component constructors.
			std::map<std::string, std::vector<int>> mEqNodeMap;
			// SvVoltage, if present, for each node (indexed starting with 0!)
			SvVoltage **mVoltages;
			// Maps the RID of a Terminal to its associated power flow
			std::map<std::string, SvPowerFlow*> mPowerFlows;
			// Number of ideal voltage sources
			int mNumVoltageSources;

			BaseComponent* mapComponent(BaseClass* obj);

			BaseComponent* mapACLineSegment(ACLineSegment* line);
			BaseComponent* mapAsynchronousMachine(AsynchronousMachine* machine);
			BaseComponent* mapEnergyConsumer(EnergyConsumer* con);
			BaseComponent* mapEquivalentInjection(EquivalentInjection* inj);
			BaseComponent* mapExternalNetworkInjection(ExternalNetworkInjection* inj);
			BaseComponent* mapPowerTransformer(PowerTransformer *trans);
			BaseComponent* mapSynchronousMachine(SynchronousMachine* machine);

			BaseComponent* newFlowPQLoad(std::string rid, std::string name);
		public:
			CIMReader(Real om);
			virtual ~CIMReader();

			bool addFile(std::string filename);
			std::vector<BaseComponent*> mapComponents();

			static double unitValue(double value, UnitMultiplier mult);
	};
};

#endif
