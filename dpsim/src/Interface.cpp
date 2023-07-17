// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>
#include <dpsim/InterfaceWorker.h>

using namespace CPS;

namespace DPsim {

	Interface::Interface(std::shared_ptr<InterfaceWorker> intf, Logger::Level logLevel, Logger::Level cliLevel, const String& name, UInt downsampling) :
		mInterfaceWorker(intf),
		mLog(Logger::get(Logger::LoggerType::SIMULATION, "interface", logLevel, cliLevel)),
		mName(name),
		mDownsampling(downsampling) {
			mQueueDpsimToInterface = std::make_shared<moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
			mQueueInterfaceToDpsim = std::make_shared<moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
			if (mInterfaceWorker)
			{
				mInterfaceWorker->mLog = mLog;
			}
	};

    void Interface::open() {
        mInterfaceWorker->open();
        mOpened = true;

        if (!mImportAttrsDpsim.empty()) {
            mInterfaceReaderThread = std::thread(Interface::ReaderThread(mQueueInterfaceToDpsim, mInterfaceWorker, mOpened));
        }
        if (!mExportAttrsDpsim.empty()) {
            mInterfaceWriterThread = std::thread(Interface::WriterThread(mQueueDpsimToInterface, mInterfaceWorker));
        }
    }

    void Interface::close() {
	    mOpened = false;
        mQueueDpsimToInterface->emplace(AttributePacket {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_CLOSE_INTERFACE
        });

        if (!mExportAttrsDpsim.empty()) {
            mInterfaceWriterThread.join();
        }

        if (!mImportAttrsDpsim.empty()) {
            mInterfaceReaderThread.join();
        }
        mInterfaceWorker->close();
    }

    CPS::Task::List Interface::getTasks() {
        auto tasks = CPS::Task::List();
        if (!mImportAttrsDpsim.empty()) {
            tasks.push_back(std::make_shared<Interface::PreStep>(*this));
        }
        if (!mExportAttrsDpsim.empty()) {
            tasks.push_back(std::make_shared<Interface::PostStep>(*this));
        }
        return tasks;
    }

    void Interface::PreStep::execute(Real time, Int timeStepCount) {
        if (timeStepCount % mIntf.mDownsampling == 0)
            mIntf.popDpsimAttrsFromQueue();
    }

    void Interface::PostStep::execute(Real time, Int timeStepCount) {
        if (timeStepCount % mIntf.mDownsampling == 0)
            mIntf.pushDpsimAttrsToQueue();
    }

    void Interface::addImport(CPS::AttributeBase::Ptr attr, bool blockOnRead, bool syncOnSimulationStart) {
        if (mOpened) {
            SPDLOG_LOGGER_ERROR(mLog, "Cannot modify interface configuration after simulation start!");
            std::exit(1);
        }

        mImportAttrsDpsim.emplace_back(attr, 0, blockOnRead, syncOnSimulationStart);
    }

    void Interface::addExport(CPS::AttributeBase::Ptr attr) {
        if (mOpened) {
            SPDLOG_LOGGER_ERROR(mLog, "Cannot modify interface configuration after simulation start!");
            std::exit(1);
        }

        mExportAttrsDpsim.emplace_back(attr, 0);
    }

    void Interface::syncImports() {
        //Block on read until all attributes with syncOnSimulationStart are read
        this->popDpsimAttrsFromQueue(true);
    }

    void Interface::syncExports() {
        //Just push all the attributes
        this->pushDpsimAttrsToQueue();
    }

    void Interface::popDpsimAttrsFromQueue(bool isSync) {
        AttributePacket receivedPacket = {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_NO_FLAGS
        };
        UInt currentSequenceId = mNextSequenceInterfaceToDpsim;

        //Wait for and dequeue all attributes that read should block on
        //The std::find_if will look for all attributes that have not been updated in the current while loop (i. e. whose sequence ID is lower than the next expected sequence ID)
        while (std::find_if(
                mImportAttrsDpsim.cbegin(),
                mImportAttrsDpsim.cend(),
                [currentSequenceId, isSync](auto attrTuple) {
                    auto &[_attr, seqId, blockOnRead, syncOnStart] = attrTuple;
                    if (isSync) {
                        return syncOnStart && seqId < currentSequenceId;
                    } else {
                        return blockOnRead && seqId < currentSequenceId;
                    }
                }) != mImportAttrsDpsim.cend()) {
            mQueueInterfaceToDpsim->wait_dequeue(receivedPacket);
            if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value)) {
                SPDLOG_LOGGER_WARN(mLog, "Failed to copy received value onto attribute in Interface!");
            }
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }

        //Fetch all remaining queue packets
        while (mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
            if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value)) {
                SPDLOG_LOGGER_WARN(mLog, "Failed to copy received value onto attribute in Interface!");
            }
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }
    }

    void Interface::pushDpsimAttrsToQueue() {
        for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
            mQueueDpsimToInterface->emplace(AttributePacket {
                std::get<0>(mExportAttrsDpsim[i])->cloneValueOntoNewAttribute(),
                i,
                std::get<1>(mExportAttrsDpsim[i]),
                AttributePacketFlags::PACKET_NO_FLAGS
            });
            std::get<1>(mExportAttrsDpsim[i]) = mCurrentSequenceDpsimToInterface;
            mCurrentSequenceDpsimToInterface++;
        }
    }

    void Interface::WriterThread::operator() () const {
        bool interfaceClosed = false;
        std::vector<Interface::AttributePacket> attrsToWrite;
        while (!interfaceClosed) {
            AttributePacket nextPacket = {
                nullptr,
                0,
                0,
                AttributePacketFlags::PACKET_NO_FLAGS
            };

            //Wait for at least one packet
            mQueueDpsimToInterface->wait_dequeue(nextPacket);
            if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                interfaceClosed = true;
            } else {
                attrsToWrite.push_back(nextPacket);
            }

            //See if there are more packets
            while(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
                if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                    interfaceClosed = true;
                } else {
                    attrsToWrite.push_back(nextPacket);
                }
            }
            mInterfaceWorker->writeValuesToEnv(attrsToWrite);
        }
    }

    void Interface::ReaderThread::operator() () const {
        std::vector<Interface::AttributePacket>  attrsRead;
        while (mOpened) {
            mInterfaceWorker->readValuesFromEnv(attrsRead);
            for (const auto &packet : attrsRead) {
                mQueueInterfaceToDpsim->enqueue(packet);
            }
            attrsRead.clear();
        }
    }

}

