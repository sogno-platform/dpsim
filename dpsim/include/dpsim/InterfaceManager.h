// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <thread>

#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim/Interface.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>

#include <readerwriterqueue.h>

namespace DPsim {

	class InterfaceManager :
		public SharedFactory<InterfaceManager> {

	public:
		typedef std::shared_ptr<InterfaceManager> Ptr;

		struct AttributePacket {
			CPS::AttributeBase::Ptr value;
			UInt attributeId; //Used to identify the attribute. Defined by the position in the `mExportAttrsDpsim` and `mImportAttrsDpsim` lists
			UInt sequenceId; //Increasing ID used to discern multiple consecutive updates of a single attribute
			unsigned char flags; //Bit 0 set: Close interface
		} typedef AttributePacket;

		enum AttributePacketFlags {
			PACKET_CLOSE_INTERFACE = 1,
		};

        InterfaceManager(Interface::Ptr intf, CPS::Logger::Log log, String name = "", bool syncOnSimulationStart = false, bool blockOnRead = false, UInt downsampling = 1) : 
			mInterface(intf),
			mLog(log),
			mName(name),
			mSyncOnSimulationStart(syncOnSimulationStart),
			mBlockOnRead(blockOnRead),
			mDownsampling(downsampling) {
				mInterface->mLog = log;
				mQueueDpsimToInterface = std::make_shared<moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
				mQueueInterfaceToDpsim = std::make_shared<moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
			};

		virtual void open();
		virtual void close();

		//Function used in the interface's simulation task to read all imported attributes from the queue
		//Called once before every simulation timestep
		virtual void pushDpsimAttrsToQueue();
		//Function used in the interface's simulation task to write all exported attributes to the queue
		//Called once after every simulation timestep
		virtual void popDpsimAttrsFromQueue();

		//Function used in the interface thread to read updated attributes from the environment and push them into the queue
		virtual void pushInterfaceAttrsToQueue() {};

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
		Interface::Ptr mInterface;
		CPS::Logger::Log mLog;
		String mName;
		bool mSyncOnSimulationStart;
		bool mBlockOnRead;
		UInt mDownsampling;
		bool mOpened = false;

		UInt mCurrentSequenceDpsimToInterface = 0;
		std::thread mInterfaceThread;

		std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueDpsimToInterface;
		std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueInterfaceToDpsim;

	public:

		class WriterThread {
			private:
				std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueDpsimToInterface;
				std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueInterfaceToDpsim;
				DPsim::Interface::Ptr mInterface;
				UInt mCurrentSequenceInterfaceToDpsim = 0;

			public:
				WriterThread(
						std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> queueDpsimToInterface,
						std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> queueInterfaceToDpsim,
				 		DPsim::Interface::Ptr intf
					) :
					mQueueDpsimToInterface(queueDpsimToInterface),
					mQueueInterfaceToDpsim(queueInterfaceToDpsim),
					mInterface(intf) {};
				void operator() ();
		};

		class PreStep : public CPS::Task {
		public:
			PreStep(InterfaceManager& intf) :
				Task(intf.mName + ".Read"), mIntf(intf) {
				for (auto attr : intf.mImportAttrsDpsim) {
					mModifiedAttributes.push_back(attr);
				}
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

