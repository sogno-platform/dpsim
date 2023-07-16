/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <map>
#include <list>

#include <dpsim-models/Filesystem.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Components.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/SimTerminal.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SystemTopology.h>


/* ====== WARNING =======
 *
 * DO NOT INCLUDE ANY CIM++ Headers here!
 *
 * CIM++ has more than 600 Headers files.
 * Including them here will also include them in <DPsim.h>
 * Which will have a huge impact on compile time!
 *
 * Use forward declarations instead!
 */


/* Forward declarations */
class CIMModel;
class BaseClass;
#ifdef CGMES_BUILD
#include <UnitMultiplier.hpp>
namespace CIMPP {
	class SvVoltage;
	class SvPowerFlow;
	class ACLineSegment;
	class SynchronousMachine;
	class ExternalNetworkInjection;
	class EnergyConsumer;
	class PowerTransformer;
	class EquivalentShunt;
	class TopologicalNode;
	class ConductingEquipment;
};
#else
#include <CIMNamespaces.hpp>
#endif

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
		IdentifiedObject::List mComponents;
		/// System frequency (has to be given to convert between reactances
		/// in CIM and inductances used inside the simulation)
		Real mFrequency;
		/// System angular frequency
		Real mOmega;
		/// Domain of the simulation components that are created by the reader
		Domain mDomain;
		/// Type of generator to be instantiated
		GeneratorType mGeneratorType;
		/// \brief Determines the overall phase configuration of the simulation.
		///
		/// This can be different from the phase type of an individual node.
		PhaseType mPhase;
		/// Maps the RID of a topological node to a PowerflowNode which holds its simulation matrix index
		/// as given in the component constructors (0 for the first node, -1 or GND for ground).
		std::map<String, TopologicalNode::Ptr> mPowerflowNodes;
		/// Maps the RID of a ConductingEquipment to a PowerflowEquipment
		std::map<String, TopologicalPowerComp::Ptr> mPowerflowEquipment;
		/// Maps the RID of a Terminal to a PowerflowTerminal
		std::map<String, TopologicalTerminal::Ptr> mPowerflowTerminals;
		///
		Bool mUseProtectionSwitches = false;

		// #### shunt component settings ####
		/// activates global shunt capacitor setting
		Bool mSetShuntCapacitor = false;
		/// global shunt capacitor value
		Real mShuntCapacitorValue = -1;
		/// activates global shunt resistor setting
		Bool mSetShuntConductance = false;
		/// global shunt resistor value
		Real mShuntConductanceValue = 1e-6;

		// #### General Functions ####
		/// Resolves unit multipliers.
		static Real unitValue(Real value, CIMPP::UnitMultiplier mult);
		///
		void processSvVoltage(CIMPP::SvVoltage* volt);
		///
		void processSvPowerFlow(CIMPP::SvPowerFlow* flow);
		///
		template<typename VarType>
		void processTopologicalNode(CIMPP::TopologicalNode* topNode);
		///
		void addFiles(const fs::path &filename);
		/// Adds CIM files to list of files to be parsed.
		void addFiles(const std::list<fs::path> &filenames);
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
		TopologicalPowerComp::Ptr mapComponent(BaseClass* obj);
		/// Returns an RX-Line.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce different models such as PI and wave model.
		TopologicalPowerComp::Ptr mapACLineSegment(CIMPP::ACLineSegment* line);
		/// Returns a transformer, either ideal or with RL elements to model losses.
		TopologicalPowerComp::Ptr mapPowerTransformer(CIMPP::PowerTransformer *trans);
		/// Returns an IdealVoltageSource with voltage setting according to load flow data
		/// at machine terminals. The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real synchronous generator models here.
		TopologicalPowerComp::Ptr mapSynchronousMachine(CIMPP::SynchronousMachine* machine);
		/// Returns an PQload with voltage setting according to load flow data.
		/// Currently the only option is to create an RL-load.
		/// The voltage should be given in kV and the angle in degree.
		/// TODO: Introduce real PQload model here.
		TopologicalPowerComp::Ptr mapEnergyConsumer(CIMPP::EnergyConsumer* consumer);
		/// Returns an external grid injection.
		TopologicalPowerComp::Ptr mapExternalNetworkInjection(CIMPP::ExternalNetworkInjection* extnet);
		/// Returns a shunt
		TopologicalPowerComp::Ptr mapEquivalentShunt(CIMPP::EquivalentShunt *shunt);

		// #### Helper Functions ####
		/// Determine base voltage associated with object
		Real determineBaseVoltageAssociatedWithEquipment(CIMPP::ConductingEquipment* equipment);

	public:
		/// Creates new reader with a name for logging.
		/// The first log level is for the reader and the second for the generated components.
		Reader(Logger::Level logLevel = Logger::Level::info,
			Logger::Level cliLevel = Logger::Level::off,
			Logger::Level componentLogLevel = Logger::Level::off);
		///
		virtual ~Reader();

		/// Parses data from CIM files into the CPS data structure
		SystemTopology loadCIM(Real systemFrequency, const fs::path &filename, Domain domain = Domain::DP, PhaseType phase = PhaseType::Single,
			GeneratorType genType = GeneratorType::None);
		/// Parses data from CIM files into the CPS data structure
		SystemTopology loadCIM(Real systemFrequency, const std::list<fs::path> &filenames, Domain domain = Domain::DP, PhaseType phase = PhaseType::Single,
			GeneratorType genType = GeneratorType::None);
		///
		SystemTopology loadCIM(Real systemFrequency, const std::list<CPS::String> &filenamesString, Domain domain = Domain::DP, PhaseType phase = PhaseType::Single,
			GeneratorType genType = GeneratorType::None);

		// #### shunt component settings ####
		/// set shunt capacitor value
		void setShuntCapacitor(Real v);
		/// set shunt conductance value
		void setShuntConductance(Real v);
		/// If set, some components like loads include protection switches
		void useProtectionSwitches(Bool value = true);
	};
}
}

#if defined(BASECLASS_H) && !defined(READER_CPP)
  #error "Do not include CIMpp headers into CPS/DPsim headers!"
#endif
