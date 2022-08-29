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

	class Interface {

	public:
		typedef std::shared_ptr<Interface> Ptr;

        Interface(bool syncOnSimulationStart = false) : mSyncOnSimulationStart(syncOnSimulationStart) { };
		virtual ~Interface() { };

		virtual void open();
		virtual void close();

		//Function used in the interface's simulation task to read all imported attributes from the queue
		//Called once before every simulation timestep
		virtual void pushDpsimAttrsToQueue();
		//Function used in the interface's simulation task to write all exported attributes to the queue
		//Called once after every simulation timestep
		virtual void popDpsimAttrsFromQueue();

		//Function used in the interface thread to read updated attributes from the environment and push them into the queue
		virtual void pushInterfaceAttrsToQueue();
		//Function used in the interface thread to read updated attributes from the queue and push them into the environment
		virtual void popInterfaceAttrsFromQueue();

		//Function that will be called on loop in its separate thread.
		//Should be used to read values from the environment and push them into `updatedAttrs`
		virtual void readValuesFromEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

		//Function that will be called on loop in its separate thread.
		//Should be used to read values from `updatedAttrs` and write them to the environment
		virtual void writeValuesToEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

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
		CPS::Logger::Log mLog;
		bool mBlockOnRead;
		bool mSyncOnSimulationStart;
		UInt mDownsampling;
		String mName;
		bool mOpened;

		moodycamel::BlockingReaderWriterQueue<CPS::AttributeBase::Ptr> mQueueDpsimToInterface;
		moodycamel::BlockingReaderWriterQueue<CPS::AttributeBase::Ptr> mQueueInterfaceToDpsim;

		//Has to be called by the interface implementation whenever a new import is configured
		virtual void importAttribute(CPS::AttributeBase::Ptr attr);
		//Has to be called by the interface implementation whenever a new export is configured
		virtual void exportAttribute(CPS::AttributeBase::Ptr attr);

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

