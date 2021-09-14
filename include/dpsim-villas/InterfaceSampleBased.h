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
#include <villas/exceptions.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

using namespace villas;

namespace DPsim {
	class InterfaceSampleBased :
		public Interface,
		public SharedFactory<InterfaceSampleBased> {

	public:
		typedef std::shared_ptr<InterfaceSampleBased> Ptr;
		typedef struct node::Sample Sample;

	protected:
		// Using std::function / lambda makes the other template code nicer, but from
		// the outside, only the attribute-based functions should be used to
		// guarantee proper scheduling

		void addImport(std::function<void(Sample*)> l) { mImports.push_back(l); }
		void addExport(std::function<void(Sample*)> l) { mExports.push_back(l); }

		std::vector<std::function<void(Sample*)>> mExports, mImports;
		CPS::AttributeBase::List mExportAttrs, mImportAttrs;

		Sample *mLastSample;
		bool mOpened;
		String mRName, mWName;
		int mSequence;

		CPS::Logger::Log mLog;

		/// Is this InterfaceVillas used for synchronization?
		bool mSync;
		/// Downsampling
		UInt mDownsampling;
		std::map<int, node::Signal::Ptr> mExportSignals;
		std::map<int, node::Signal::Ptr> mImportSignals;

	public:

		class PreStep : public CPS::Task {
		public:
			PreStep(InterfaceSampleBased& intf) :
				Task(intf.mRName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrs) {
					mModifiedAttributes.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceSampleBased& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			PostStep(InterfaceSampleBased& intf) :
				Task(intf.mWName+ ".Write"), mIntf(intf) {
				for (auto attr : intf.mExportAttrs) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceSampleBased& mIntf;
		};

		InterfaceSampleBased(const String &wn, const String &rn, Bool sync = true, UInt downsampling = 1) :
            mOpened(false),
            mRName(rn),
            mWName(wn),
	        mSync(sync),
	        mDownsampling(downsampling) {}
			
		virtual ~InterfaceSampleBased() {
			if (mOpened)
				close();
		}

		virtual void open(CPS::Logger::Log log) = 0;
		virtual void close();

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
		virtual void readValues(bool blocking = true) = 0;

		/** Write all exported values to the InterfaceVillas. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		virtual void writeValues() = 0;

		CPS::Task::List getTasks();
	};
}

