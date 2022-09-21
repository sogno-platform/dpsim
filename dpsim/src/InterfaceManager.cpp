// SPDX-License-Identifier: Apache-2.0

#include <dpsim/InterfaceManager.h>

using namespace CPS;

namespace DPsim {

    void InterfaceManager::open() {
        mInterfaceThread = std::thread(InterfaceManager::WriterThread(mQueueDpsimToInterface, mQueueInterfaceToDpsim, mInterface));
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
        mInterfaceThread.join();
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
            //UNIMPLEMENTED
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
        CPS::AttributeBase::List attrsRead;
        mInterface->open();
        while (!interfaceClosed) {

            mInterface->readValuesFromEnv(attrsRead);
            for (unsigned int i = 0; i < attrsRead.size(); i++) {
                mQueueInterfaceToDpsim->enqueue(AttributePacket {
                    attrsRead[i],
                    i,
                    mCurrentSequenceInterfaceToDpsim++,
                    0
                });
            }
            attrsRead.clear();

            AttributePacket nextPacket;
            if(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
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

}

