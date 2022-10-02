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

    void InterfaceManager::importAttribute(CPS::AttributeBase::Ptr attr) {
        mImportAttrsDpsim.push_back(attr);
    }

    void InterfaceManager::exportAttribute(CPS::AttributeBase::Ptr attr) {
        mExportAttrsDpsim.push_back(attr);
    }

    void InterfaceManager::popDpsimAttrsFromQueue() {
        if (mBlockOnRead) {
            AttributePacket receivedPacket;
            //TODO: This will only wait for ONE packet, not for all requested attributes
            mQueueInterfaceToDpsim->wait_dequeue(receivedPacket);
            mImportAttrsDpsim[receivedPacket.attributeId]->copyValue(receivedPacket.value);
        } else {
            AttributePacket receivedPacket;
            if(mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
                mImportAttrsDpsim[receivedPacket.attributeId]->copyValue(receivedPacket.value);
            }
        }
    }

    void InterfaceManager::pushDpsimAttrsToQueue() {
        for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
            mQueueDpsimToInterface->enqueue(AttributePacket {
                mExportAttrsDpsim[i]->cloneValueOntoNewAttribute(),
                i,
                mCurrentSequenceDpsimToInterface++,
                0
            });
        }
    }

    void InterfaceManager::WriterThread::operator() () {
        bool interfaceClosed = false;
        CPS::AttributeBase::List attrsToWrite;
        while (!interfaceClosed) {
            AttributePacket nextPacket;
            if(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
                if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                    interfaceClosed = true;
                } else {
                    attrsToWrite.push_back(nextPacket.value); //TODO: The interface should know about the attribute and sequence IDs
                    mInterface->writeValuesToEnv(attrsToWrite);
                }
            }
            
        }
    }

    void InterfaceManager::ReaderThread::operator() () {
        CPS::AttributeBase::List attrsRead;
        while (mOpened) { //TODO: As long as reading blocks, there is no real way to force-stop thread execution from the dpsim side
            mInterface->readValuesFromEnv(attrsRead);
            for (unsigned int i = 0; i < attrsRead.size(); i++) {
                mQueueInterfaceToDpsim->enqueue(AttributePacket {
                    attrsRead[i],
                    i, //TODO: The attribute ID should be provided by the interface
                    mCurrentSequenceInterfaceToDpsim++,
                    0
                });
            }
            attrsRead.clear();
        }
    }

}

