/** Read CIM files
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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

#pragma once

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
			// All components after mapping
			std::vector<BaseComponent*> mComponents;
			// System frequency (has to be given to convert between reactances
			// in CIM and inductances used inside the simulation)
			Real mFrequency;
			// Maps the RID of a topological node to its simulation matrix index
			// as given in the component constructors (1 for the first node).
			std::map<std::string, Int> mTopNodes;
			// Maps the RID of a ConductingEquipment to a list of nodes as given in
			// the component constructors.
			std::map<std::string, std::vector<Int>> mEqNodeMap;
			// SvVoltage, if present, for each node (indexed starting with 0!)
			SvVoltage **mVoltages;
			// Maps the RID of a Terminal to its associated power flow
			std::map<std::string, SvPowerFlow*> mPowerFlows;
			// Number of ideal voltage sources
			Int mNumVoltageSources;

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
			void parseFiles();
			std::vector<BaseComponent*>& getComponents();
			Int mapTopologicalNode(std::string mrid);
			Int getNumVoltageSources();

			static Real unitValue(Real value, UnitMultiplier mult);
	};
};
