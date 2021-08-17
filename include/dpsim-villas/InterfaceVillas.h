/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

// #############################################
// Do NOT include this header in any MPL2 files
// #############################################

#pragma once

#include <vector>
#include <map>

#include <cps/PtrFactory.h>
#include <dpsim/Interface.h>

#include <villas/sample.hpp>
#include <villas/node.hpp>

using namespace villas;

namespace DPsim {
	class InterfaceVillas :
		public Interface,
		public SharedFactory<InterfaceVillas> {

	public:
		typedef std::shared_ptr<InterfaceVillas> Ptr;
		typedef struct node::Sample Sample;

	protected:
		// Using std::function / lambda makes the other template code nicer, but from
		// the outside, only the attribute-based functions should be used to
		// guarantee proper scheduling

		void addImport(std::function<void(Sample*)> l) { mImports.push_back(l); }
		void addExport(std::function<void(Sample*)> l) { mExports.push_back(l); }

		std::vector<std::function<void(Sample*)>> mExports, mImports;
		CPS::AttributeBase::List mExportAttrs, mImportAttrs;

		//VillasNode instance
		//node::Node mNode;
		Sample *mLastSample;
		String mName;
		bool mOpened;
		int mSequence;

		CPS::Logger::Log mLog;

		/// Is this InterfaceVillas used for synchronization?
		bool mSync;
		/// Downsampling
		UInt mDownsampling;

		class Signal {
			public:
			std::string mName;
			std::string mUnit;
			node::SignalType mType;

			Signal() {}
			Signal(UInt idx, enum node::SignalType type, const std::string &name="", const std::string &unit="") :
				mName(name),
				mUnit(unit),
				mType(type) {

				if (mName.empty())
					mName = fmt::format("signal_{}", idx);
			}
		};

		std::map<int, Signal> mExportSignals;

	public:

		class PreStep : public CPS::Task {
		public:
			PreStep(InterfaceVillas& intf) :
				Task(intf.mName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrs) {
					mModifiedAttributes.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceVillas& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			PostStep(InterfaceVillas& intf) :
				Task(intf.mName+ ".Write"), mIntf(intf) {
				for (auto attr : intf.mExportAttrs) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceVillas& mIntf;
		};

		/** Create a InterfaceVillas with a specific configuration for the VillasNode
		 *
		 * @param name The name of the newly created VillasNode
		 */
		InterfaceVillas(const String &name, Bool sync = true, UInt downsampling = 1) :
			mName(name),
			mOpened(false),
			mSync(sync),
			mDownsampling(downsampling)
		{
		}

		~InterfaceVillas() {
			if (mOpened)
				close();
		}

		void open(CPS::Logger::Log log);
		void close();

		CPS::Attribute<Int>::Ptr importInt(UInt idx);
		CPS::Attribute<Real>::Ptr importReal(UInt idx);
		CPS::Attribute<Bool>::Ptr importBool(UInt idx);
		CPS::Attribute<Complex>::Ptr importComplex(UInt idx);
		CPS::Attribute<Complex>::Ptr importComplexMagPhase(UInt idx);

		void exportInt(CPS::Attribute<Int>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportReal(CPS::Attribute<Real>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportBool(CPS::Attribute<Bool>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");
		void exportComplex(CPS::Attribute<Complex>::Ptr attr, UInt idx, const std::string &name="", const std::string &unit="");

		/** Read data for a timestep from the InterfaceVillas and passes the values
		 * to all registered current / voltage sources.
		 */
		void readValues(bool blocking = true);

		/** Write all exported values to the InterfaceVillas. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		void writeValues();

		CPS::Task::List getTasks();
	};
}

