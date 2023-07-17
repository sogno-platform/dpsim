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

	class InterfaceWorker;

	class Interface :
		public SharedFactory<Interface> {

	public:
		typedef std::shared_ptr<Interface> Ptr;

		using AttributePacket = struct AttributePacket {
			CPS::AttributeBase::Ptr value;
			UInt attributeId; //Used to identify the attribute. Defined by the position in the `mExportAttrsDpsim` and `mImportAttrsDpsim` lists
			UInt sequenceId; //Increasing ID used to discern multiple consecutive updates of a single attribute
			unsigned char flags; //Bit 0 set: Close interface
		};

		enum AttributePacketFlags {
			PACKET_NO_FLAGS = 0,
			PACKET_CLOSE_INTERFACE = 1,
		};

        Interface(std::shared_ptr<InterfaceWorker> intf, CPS::Logger::Level fileLevel = CPS::Logger::Level::info, CPS::Logger::Level cliLevel = CPS::Logger::Level::info, const String& name = "", UInt downsampling = 1);

		virtual void open();
		virtual void close();

		// Function used in the interface's simulation task to read all imported attributes from the queue
		// Called once before every simulation timestep
		virtual void pushDpsimAttrsToQueue();
		// Function used in the interface's simulation task to write all exported attributes to the queue
		// Called once after every simulation timestep
		virtual void popDpsimAttrsFromQueue(bool isSync = false);

		// Function called by the Simulation to perform interface synchronization
		virtual void syncExports();
		/// Function called by the Simulation to perform interface synchronization
		virtual void syncImports();

		virtual CPS::Task::List getTasks();

		virtual ~Interface() {
			if (mOpened)
				close();
		}

		// Attributes used in the DPsim simulation. Should only be accessed by the dpsim-thread
		// Tuple attributes: Attribute to be imported, Current sequenceID, blockOnRead, syncOnSimulationStart
		std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt, bool, bool>> mImportAttrsDpsim;
		// Tuple attributes: Attribute to be exported, Current Sequence ID
		std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt>> mExportAttrsDpsim;

	protected:
		std::shared_ptr<InterfaceWorker> mInterfaceWorker;
		CPS::Logger::Log mLog;
		String mName;
		bool mSyncOnSimulationStart;
		UInt mCurrentSequenceDpsimToInterface = 1;
		UInt mNextSequenceInterfaceToDpsim = 1;
		UInt mDownsampling;
		std::atomic<bool> mOpened;
		std::thread mInterfaceWriterThread;
		std::thread mInterfaceReaderThread;

		std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueDpsimToInterface;
		std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueInterfaceToDpsim;

		virtual void addImport(CPS::AttributeBase::Ptr attr, bool blockOnRead = false, bool syncOnSimulationStart = true);
		virtual void addExport(CPS::AttributeBase::Ptr attr);

	public:

		class WriterThread {
			private:
				std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueDpsimToInterface;
				std::shared_ptr<InterfaceWorker> mInterfaceWorker;

			public:
				WriterThread(
						std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> queueDpsimToInterface,
				 		std::shared_ptr<InterfaceWorker> intf
					) :
					mQueueDpsimToInterface(queueDpsimToInterface),
					mInterfaceWorker(intf) {};
				void operator() () const;
		};

		class ReaderThread {
			private:
				std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> mQueueInterfaceToDpsim;
				std::shared_ptr<InterfaceWorker> mInterfaceWorker;
				std::atomic<bool>& mOpened;

			public:
				ReaderThread(
						std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>> queueInterfaceToDpsim,
				 		std::shared_ptr<InterfaceWorker> intf,
						std::atomic<bool>& opened
					) :
					mQueueInterfaceToDpsim(queueInterfaceToDpsim),
					mInterfaceWorker(intf),
					mOpened(opened) {};
				void operator() () const;
		};

		class PreStep : public CPS::Task {
		public:
			explicit PreStep(Interface& intf) :
				Task(intf.mName + ".Read"), mIntf(intf) {
				for (const auto& [attr, _seqId, _blockOnRead, _syncOnStart] : intf.mImportAttrsDpsim) {
					mModifiedAttributes.push_back(attr);
				}
			}

			void execute(Real time, Int timeStepCount) override;

		private:
			Interface& mIntf;
		};

		class PostStep : public CPS::Task {
		public:
			explicit PostStep(Interface& intf) :
				Task(intf.mName + ".Write"), mIntf(intf) {
				for (const auto& [attr, _seqId] : intf.mExportAttrsDpsim) {
					mAttributeDependencies.push_back(attr);
				}
				mModifiedAttributes.push_back(Scheduler::external);
			}

			void execute(Real time, Int timeStepCount) override;

		private:
			Interface& mIntf;
		};

	};
}

