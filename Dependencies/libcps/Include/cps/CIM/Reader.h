/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
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
#include <list>
#include <experimental/filesystem>
#include <cps/Definitions.h>
#include <cps/PowerComponent.h>
#include <cps/Components.h>
#include <cps/Node.h>
#include <cps/Terminal.h>
#include <cps/Logger.h>
#include <cps/SystemTopology.h>

/* ====== WARNING =======
 *
 * DO NOT INCLUDE ANY CIM++ Headers here!
 *
 * CIM++ consists over 600 Headers files.
 * Including them here will also include them in <DPsim.h>
 * Which will have a huge impact on compile time!
 *
 * Use forward declarations instead!
 */

#include <IEC61970/Base/Domain/UnitMultiplier.h>

/* Forward declarations */
class CIMModel;
class BaseClass;
namespace IEC61970 {
	namespace Base {
		namespace StateVariables {
			class SvVoltage;
			class SvPowerFlow;
		};
		namespace Wires {
			class ACLineSegment;
			class SynchronousMachine;
			class ExternalNetworkInjection;
			class EnergyConsumer;
			class PowerTransformer;
		};
		namespace Equivalents {
			class EquivalentShunt;
		}
		namespace Topology {
			class TopologicalNode;
		};
	};
};

namespace CPS {
namespace CIM {
	class InvalidTopology { };

	class Reader {
	private:
		/// CIM logger
		Logger::Log mSLog;
		/// Log level of components
		Logger::Level mComponentLogLevel;
		/// Model from CIM++
		CIMModel *mModel;
		/// All components after mapping
		Component::List mComponents;
		/// System frequency (has to be given to convert between reactances
		/// in CIM and inductances used inside the simulation)
		Real mFrequency;
		/// System angular frequency
		Real mOmega;
		/// Domain of the simulation components that are created by the reader
		Domain mDomain;
		/// \brief Determines the overall phase configuration of the simulation.
		///
		/// This can be different from the phase type of an individual node.
		PhaseType mPhase;
		/// Maps the RID of a topological node to a PowerflowNode which holds its simulation matrix index
		/// as given in the component constructors (0 for the first node, -1 or GND for ground).
		std::map<String, TopologicalNode::Ptr> mPowerflowNodes;
		/// Maps the RID of a ConductingEquipment to a PowerflowEquipment
		std::map<String, Component::Ptr> mPowerflowEquipment;
		/// Maps the RID of a Terminal to a PowerflowTerminal
		std::map<String, TopologicalTerminal::Ptr> mPowerflowTerminals;
		///

		// #### General Functions ####
		/// Resolves unit multipliers.
		static Real unitValue(Real value, IEC61970::Base::Domain::UnitMultiplier mult);
		///
		Matrix singlePhaseParameterToThreePhase(Real parameter);
		///
		void processSvVoltage(IEC61970::Base::StateVariables::SvVoltage* volt);
		///
		void processSvPowerFlow(IEC61970::Base::StateVariables::SvPowerFlow* flow);
		///
		template<typename VarType>
		void processTopologicalNode(IEC61970::Base::Topology::TopologicalNode* topNode);
		///
		void addFiles(const std::experimental::filesystem::path &filename);
		/// Adds CIM files to list of files to be parsed.
		void addFiles(const std::list<std::experimental::filesystem::path> &filenames);
		/// First, go through all topological nodes and collect them in a list.
		/// Since all nodes have references to the equipment connected to them (via Terminals), but not
		/// the other way around (which we need for instantiating the components), we collect that information here as well.
		void parseFiles();
		/// Returns list of components and nodes.
		SystemTopology systemTopology();

		// #### Mapping Functions ####
		/// Returns simulation node index which belongs to mRID.
		Matrix::Index mapTopologicalNode(String mrid);
		/// Maps CIM components to CPowerSystem components.
		Component::Ptr mapComponent(BaseClass* obj);
		/// Returns an RX-Line.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce different models such as PI and wave model.
		Component::Ptr mapACLineSegment(IEC61970::Base::Wires::ACLineSegment* line);
		/// Returns a transformer, either ideal or with RL elements to model losses.
		Component::Ptr mapPowerTransformer(IEC61970::Base::Wires::PowerTransformer *trans);
		/// Returns an IdealVoltageSource with voltage setting according to load flow data
		/// at machine terminals. The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real synchronous generator models here.
		Component::Ptr mapSynchronousMachine(IEC61970::Base::Wires::SynchronousMachine* machine);
		/// Returns an PQload with voltage setting according to load flow data.
		/// Currently the only option is to create an RL-load.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real PQload model here.
		Component::Ptr mapEnergyConsumer(IEC61970::Base::Wires::EnergyConsumer* consumer);
		/// Adds CIM files to list of files to be parsed.

		/// Returns an external grid injection.
		Component::Ptr mapExternalNetworkInjection(IEC61970::Base::Wires::ExternalNetworkInjection* extnet);
		/// Returns a shunt
		Component::Ptr mapEquivalentShunt(IEC61970::Base::Equivalents::EquivalentShunt *shunt);
	public:
		///
		enum GeneratorType{Static, Transient};
		///
		GeneratorType mGeneratorType = GeneratorType::Transient;
		/// Creates new reader with a name for logging.
		/// The first log level is for the reader and the second for the generated components.
		Reader(String name,
			Logger::Level logLevel = Logger::Level::off,
			Logger::Level componentLogLevel = Logger::Level::off);
		///
		virtual ~Reader();
		/// Parses data from CIM files into the CPS data structure
		SystemTopology loadCIM(Real systemFrequency, const std::experimental::filesystem::path &filename, Domain domain = Domain::DP, PhaseType phase = PhaseType::Single);
		/// Parses data from CIM files into the CPS data structure
		SystemTopology loadCIM(Real systemFrequency, const std::list<std::experimental::filesystem::path> &filenames, Domain domain = Domain::DP, PhaseType phase = PhaseType::Single);
		/// writing state variable voltage of nodes from the another SystemTopology, which has the same topology but modeled in different domain
		void writeSvVoltageFromStaticSysTopology(SystemTopology& sysStatic, SystemTopology& sysDynamic);
	};
}
}

// Allow inclusion of CIMpp headers only from Reader.cpp
// For performance reasons
#if defined(BASECLASS_H) && !defined(READER_CPP)
  #error "Do not include CIMpp headers into CPS/DPsim headers!"
#endif
