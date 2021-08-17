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
#include <villas/shmem.hpp>

using namespace villas;

namespace DPsim {
	/// Shmem interface used in combination with VILLAS
	class InterfaceShmem :
		public Interface,
		public SharedFactory<InterfaceShmem> {

	public:
		typedef std::shared_ptr<InterfaceShmem> Ptr;
		typedef struct node::Sample Sample;
		typedef struct node::ShmemConfig Config;
		typedef struct node::ShmemInterface ShmemInt;

	protected:
		// Using std::function / lambda makes the other template code nicer, but from
		// the outside, only the attribute-based functions should be used to
		// guarantee proper scheduling

		void addImport(std::function<void(Sample*)> l) { mImports.push_back(l); }
		void addExport(std::function<void(Sample*)> l) { mExports.push_back(l); }

		std::vector<std::function<void(Sample*)>> mExports, mImports;
		CPS::AttributeBase::List mExportAttrs, mImportAttrs;

		ShmemInt mShmem;
		Sample *mLastSample;

		bool mOpened;
		int mSequence;
		String mRName, mWName;
		Config mConf;

		CPS::Logger::Log mLog;

		/// Is this InterfaceShmem used for synchorinzation?
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
			PreStep(InterfaceShmem& intf) :
				Task(intf.mRName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrs) {
					mModifiedAttributes.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceShmem& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			PostStep(InterfaceShmem& intf) :
				Task(intf.mWName + ".Write"), mIntf(intf) {
				for (auto attr : intf.mExportAttrs) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceShmem& mIntf;
		};

		/** Create a InterfaceShmem with a specific configuration for the output queue.
		 *
		 * @param wname The name of the POSIX shmem object where samples will be written to.
		 * @param rname The name of the POSIX shmem object where samples will be read from.
		 * @param conf The configuration object for the output queue (see VILLASnode's documentation), or nullptr for sensible defaults.
		 */
		InterfaceShmem(const String &wn, const String &rn, Config *conf = nullptr, Bool sync = true, UInt downsampling = 1) :
			mOpened(false),
			mRName(rn),
			mWName(wn),
			mSync(sync),
			mDownsampling(downsampling)
		{
			if (conf != nullptr) {
				mConf = *conf;
			} else {
				mConf.queuelen = 512;
				mConf.samplelen = 64;
				mConf.polling = 0;
			}
		}

		~InterfaceShmem() {
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

		/** Read data for a timestep from the InterfaceShmem and passes the values
		 * to all registered current / voltage sources.
		 */
		void readValues(bool blocking = true);

		/** Write all exported values to the InterfaceShmem. Called after every timestep.
		 * @param model Reference to the system model which should be used to
		 * calculate needed voltages.
		 */
		void writeValues();

		CPS::Task::List getTasks();
	};
}

