// SPDX-License-Identifier: Apache-2.0

#include <dpsim/InterfaceManager.h>

using namespace CPS;

namespace DPsim {

    void InterfaceManager::open() {
        mInterface->open();
        mInterfaceWriterThread = std::thread(InterfaceManager::WriterThread(mQueueDpsimToInterface, mInterface));
        mInterfaceReaderThread = std::thread(InterfaceManager::ReaderThread(mQueueInterfaceToDpsim, mInterface, mOpened));
        mOpened = true;
    }

    void InterfaceManager::close() {
	    mOpened = false;
        mQueueDpsimToInterface->enqueue(AttributePacket {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_CLOSE_INTERFACE
        });
        mInterfaceWriterThread.join();
        mInterfaceReaderThread.join();
        mInterface->close();
    }

    CPS::Task::List InterfaceManager::getTasks() {
        return CPS::Task::List({
            std::make_shared<InterfaceManager::PreStep>(*this),
            std::make_shared<InterfaceManager::PostStep>(*this)
        });
    }

    void InterfaceManager::PreStep::execute(Real time, Int timeStepCount) {
        if (!mIntf.mImportAttrsDpsim.empty()) {
            if (timeStepCount % mIntf.mDownsampling == 0)
                mIntf.popDpsimAttrsFromQueue();
        }	
    }

    void InterfaceManager::PostStep::execute(Real time, Int timeStepCount) {
        if (!mIntf.mExportAttrsDpsim.empty()) {
            if (timeStepCount % mIntf.mDownsampling == 0)
                mIntf.pushDpsimAttrsToQueue();
        }
    }

    void InterfaceManager::importAttribute(CPS::AttributeBase::Ptr attr, bool blockOnRead) {
        mImportAttrsDpsim.push_back(std::make_tuple(attr, 0, blockOnRead));
        //mInterface->configureImport(mImportAttrsDpsim.size() - 1, attr->getType());
    }

    void InterfaceManager::exportAttribute(CPS::AttributeBase::Ptr attr) {
        mExportAttrsDpsim.push_back(std::make_tuple(attr, 0));
        //mInterface->configureExport(mImportAttrsDpsim.size() - 1, attr->getType());
    }

    void InterfaceManager::popDpsimAttrsFromQueue() {
        AttributePacket receivedPacket;
        UInt currentSequenceId = mCurrentSequenceInterfaceToDpsim;

        //Wait for and dequeue all attributes that read should block on
        //The std::find_if will look for all attributes that have not been updated in the current while loop (i. e. whose sequence ID is lower than the next expected sequence ID)
        while (std::find_if(
                mImportAttrsDpsim.cbegin(),
                mImportAttrsDpsim.cend(),
                [](auto attrTuple) {
                    return std::get<2>(attrTuple) && std::get<1>(attrTuple) < currentSequenceId
                }) != mImportAttrsDpsim.cend()) {
            mQueueInterfaceToDpsim->wait_dequeue(receivedPacket);
            std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value);
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mCurrentSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }

        //Fetch all remaining queue packets
        while (mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
            std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value);
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mCurrentSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }
    }

    void InterfaceManager::pushDpsimAttrsToQueue() {
        for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
            mQueueDpsimToInterface->enqueue(AttributePacket {
                std::get<0>(mExportAttrsDpsim[i])->cloneValueOntoNewAttribute(),
                i,
                std::get<1>(mExportAttrsDpsim[i]),
                AttributePacketFlags::PACKET_NO_FLAGS
            });
            std::get<1>(mExportAttrsDpsim[i]) = mCurrentSequenceDpsimToInterface;
            mCurrentSequenceDpsimToInterface++;
        }
    }

    void InterfaceManager::WriterThread::operator() () {
        bool interfaceClosed = false;
        std::vector<InterfaceManager::AttributePacket> attrsToWrite;
        while (!interfaceClosed) {
            AttributePacket nextPacket;
            if(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
                if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                    interfaceClosed = true;
                } else {
                    attrsToWrite.push_back(nextPacket);
                    mInterface->writeValuesToEnv(attrsToWrite);
                }
            }
            
        }
    }

    void InterfaceManager::ReaderThread::operator() () {
        std::vector<InterfaceManager::AttributePacket>  attrsRead;
        while (mOpened) { //TODO: As long as reading blocks, there is no real way to force-stop thread execution from the dpsim side
            mInterface->readValuesFromEnv(attrsRead);
            for (auto packet : attrsRead) {
                mQueueInterfaceToDpsim->enqueue(packet);
            }
            attrsRead.clear();
        }
    }

}

