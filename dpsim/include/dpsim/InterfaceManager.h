// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim/Interface.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>

#include <readerwriterqueue.h>

namespace DPsim {

	class InterfaceManager {

	public:
		typedef std::shared_ptr<InterfaceManager> Ptr;

		struct AttributePacket {
			CPS::AttributeBase::Ptr value;
			UInt attributeId;
			UInt sequenceId;
		} typedef AttributePacket;

        InterfaceManager(bool syncOnSimulationStart = false) : mSyncOnSimulationStart(syncOnSimulationStart) { };
		virtual ~InterfaceManager() { };

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

		virtual CPS::Task::List getTasks();

		bool shouldSyncOnSimulationStart() const {
			return mSyncOnSimulationStart;
		}

		virtual ~InterfaceManager() {
			if (mOpened)
				close();
		}

		// Attributes used in the DPsim simulation. Should only be accessed by the dpsim-thread
		CPS::AttributeBase::List mExportAttrsDpsim, mImportAttrsDpsim;

		//Has to be called by the interface implementation whenever a new import is configured
		virtual void importAttribute(CPS::AttributeBase::Ptr attr);
		//Has to be called by the interface implementation whenever a new export is configured
		virtual void exportAttribute(CPS::AttributeBase::Ptr attr);

	protected:
		CPS::Logger::Log mLog;
		bool mBlockOnRead;
		bool mSyncOnSimulationStart;
		UInt mDownsampling;
		String mName;
		bool mOpened = false;

		UInt mReceivedIdsDpsim = 0;

		UInt mCurrentSequenceDpsimToInterface = 0;
		UInt mCurrentSequenceInterfaceToDpsim = 0;

		Interface::Ptr mInterface;

		moodycamel::BlockingReaderWriterQueue<AttributePacket> mQueueDpsimToInterface;
		moodycamel::BlockingReaderWriterQueue<AttributePacket> mQueueInterfaceToDpsim;

	public:

		class PreStep : public CPS::Task {
		public:
			PreStep(InterfaceManager& intf) :
				Task(intf.mName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrsDpsim) {
					mModifiedAttributes.push_back(attr);
				}
				//TODO: Is this necessary / what effect does a dependency on external have?
				mAttributeDependencies.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceManager& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			PostStep(InterfaceManager& intf) :
				Task(intf.mName + ".Write"), mIntf(intf) {
				for (auto attr : intf.mExportAttrsDpsim) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount);

		private:
			InterfaceManager& mIntf;
		};

	};
}

