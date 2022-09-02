// SPDX-License-Identifier: Apache-2.0

#include <dpsim/InterfaceManager.h>

using namespace CPS;

namespace DPsim {

    void InterfaceManager::open() {
        mInterfaceThread = std::thread(InterfaceManager::WriterThread(mQueueDpsimToInterface, mInterface));
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
    }

    CPS::Task::List InterfaceManager::getTasks() {
        //TODO: Should this only be two tasks (reading + writing) or one task for every attribute that is imported / exported?
        //Due to the dependencies on external, it should not matter --> verify this behavior?
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
            //UNIMPLEMENTED
        } else {
            AttributePacket receivedPacket;
            while (mQueueDpsimToInterface->try_dequeue(receivedPacket)) {
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
        mInterface->open();
        while (!interfaceClosed) {
            AttributePacket nextPacket;
            mQueueDpsimToInterface->wait_dequeue(nextPacket);
            if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                mInterface->close();
                interfaceClosed = true;
            } else {
                attrsToWrite.push_back(nextPacket.value); //TODO: The interface should know about the attribute and sequence IDs
                mInterface->writeValuesToEnv(attrsToWrite);
            }
        }
    }
}

