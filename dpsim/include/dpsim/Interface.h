// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>

#include <readerwriterqueue.h>

namespace DPsim {

	/**
	 * After an Interface is created, components that should use values
	 * from this interface can be registered with it using the appropiate
	 * methods implemented by the subclass. Subclasses must also implement the
	 * readValues and writeValues methods, which should update the values of
	 * the registered components or send voltages or currents to the external
	 * sink.
	 */

	//TODO: Expand this to add reading and writing from the queue
	class Interface {

	public:
		typedef std::shared_ptr<Interface> Ptr;

        Interface(bool syncOnSimulationStart = false) : mSyncOnSimulationStart(syncOnSimulationStart) { };
		virtual ~Interface() { };

		virtual void open(CPS::Logger::Log log) = 0;
		virtual void close() = 0;

		virtual void importAttribute(CPS::AttributeBase::Ptr attr);
		virtual void exportAttribute(CPS::AttributeBase::Ptr attr);

		//Function used in the interface's simulation task to read all imported attributes from the queue
		//Called once before every simulation timestep
		virtual void readValuesFromQueue();
		//Function used in the interface's simulation task to write all exported attributes to the queue
		//Called once after every simulation timestep
		virtual void writeValuesToQueue();

		//Function that will be called on loop in its separate thread.
		//Should be used to read values from the environment and push them into the queue
		virtual void readValuesFromEnv() = 0;

		//Function that will be called on loop in its separate thread.
		//Should be used to read values from the queue and write them to the environment
		virtual void writeValuesToEnv() = 0;

		//Function that will be called once on the dpsim thread before starting the interface thread
		//Can be used for mapping the attributes in `mExportAttrsDpsim` and `mImportAttrsDpsim` to interface import / exports
		virtual void prepareInterfaceThread() = 0;

		virtual CPS::Task::List getTasks();

		bool shouldSyncOnSimulationStart() const {
			return mSyncOnSimulationStart;
		}

		virtual ~Interface() {
			if (mOpened)
				close();
		}

		// Attributes used in the DPsim simulation. Should only be accessed by the dpsim-thread
		CPS::AttributeBase::List mExportAttrsDpsim, mImportAttrsDpsim;

	protected:
		// Attributes used by the interface thread for importing and exporting
		CPS::AttributeBase::List mExportAttrsInterface, mImportAttrsInterface;
		CPS::Logger::Log mLog;
		bool mBlockOnRead;
		bool mSyncOnSimulationStart;
		UInt mDownsampling;
		String mName;
		bool mOpened;

	public:
		class PreStep : public CPS::Task {
		public:
			PreStep(Interface& intf) :
				Task(intf.mName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrsDpsim) {
					mModifiedAttributes.push_back(attr);
				}
				//TODO: Is this necessary / what effect does a dependency on external have?
				mAttributeDependencies.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			Interface& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			PostStep(Interface& intf) :
				Task(intf.mName + ".Write"), mIntf(intf) {
				for (auto attr : intf.mExportAttrsDpsim) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			Interface& mIntf;
		};

	};
}

