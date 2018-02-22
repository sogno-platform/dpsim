/** Read CIM files
 *
 * @file
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

#pragma once

#include <map>
#include <vector>

// CIMpp includes
#include <CIMModel.hpp>
#include <IEC61970.hpp>

#include "Definitions.h"
#include "Component.h"
#include "Node.h"
#include "Terminal.h"

#include "Logger.h"
#include "Simulation.h"

using namespace IEC61970::Base::Domain;
using namespace IEC61970::Base::Equivalents;
using namespace IEC61970::Base::StateVariables;
using namespace IEC61970::Base::Topology;
using namespace IEC61970::Base::Wires;

namespace DPsim {
namespace CIM {
	
	class Reader {
	private:
		/// CIM logger
		Logger mLog;
		/// Log level of components
		Logger::Level mComponentLogLevel;
		/// Model from CIM++
		CIMModel mModel;
		/// All components after mapping
		Component::List mComponents;
		/// System frequency (has to be given to convert between reactances
		/// in CIM and inductances used inside the simulation)
		Real mFrequency;
		/// Maps the RID of a topological node to a PowerflowNode which holds its simulation matrix index
		/// as given in the component constructors (0 for the first node, -1 or GND for ground).
		std::map<String, std::shared_ptr<Node>> mPowerflowNodes;
		/// Maps the RID of a ConductingEquipment to a PowerflowEquipment
		std::map<String, std::shared_ptr<Component>> mPowerflowEquipment;
		/// Maps the RID of a Terminal to a PowerflowTerminal
		std::map<String, std::shared_ptr<Terminal>> mPowerflowTerminals;

		/// Resolves unit multipliers.
		static Real unitValue(Real value, UnitMultiplier mult);

		void processTopologicalNode(TopologicalNode* topNode);

		void processSvVoltage(SvVoltage* volt);

		void processSvPowerFlow(SvPowerFlow* flow);

		/// Returns simulation node index which belongs to mRID.
		Matrix::Index mapTopologicalNode(String mrid);		
		/// Maps CIM components to DPsim components.
		Component::Ptr mapComponent(BaseClass* obj);		
		/// Returns an RX-Line.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce different models such as PI and wave model.
		Component::Ptr mapACLineSegment(ACLineSegment* line);			
		/// Returns a transformer, either ideal or with RL elements to model losses.
		Component::Ptr mapPowerTransformer(PowerTransformer *trans);
		/// Returns an IdealVoltageSource with voltage setting according to load flow data
		/// at machine terminals. The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real synchronous generator models here.
		Component::Ptr mapSynchronousMachine(SynchronousMachine* machine);
		/// Returns an PQload with voltage setting according to load flow data.
		/// Currently the only option is to create an RL-load.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real PQload model here.
		Component::Ptr mapEnergyConsumer(EnergyConsumer* consumer);
		/// Not tested yet.
		Component::Ptr mapExternalNetworkInjection(ExternalNetworkInjection* inj);		
		/// Not implemented yet.
		void mapEquivalentInjection(EquivalentInjection* inj) {}
	public:
		Reader(Real om, Logger::Level logLevel = Logger::Level::NONE, Logger::Level componentLogLevel = Logger::Level::NONE);
		virtual ~Reader();
		/// Adds CIM files to list of files to be parsed.
		bool addFile(String filename);
		/// First, go through all topological nodes and collect them in a list.
		/// Since all nodes have references to the equipment connected to them (via Terminals), but not
		/// the other way around (which we need for instantiating the components), we collect that information here as well.
		void parseFiles();
		/// Returns list of components.
		Component::List& getComponents();
	};
}
}
