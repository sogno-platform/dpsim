// SPDX-License-Identifier: Apache-2.0

#include <dpsim/InterfaceManager.h>

using namespace CPS;

namespace DPsim {

    void InterfaceManager::open() {
        mOpened = true;
    }

    void InterfaceManager::close() {
        //TODO: Close threads / queue
	    mOpened = false;
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
        if (attr->isStatic()) {
            mLog->error("Cannot import to a static attribute. Please provide a dynamic attribute!");
            throw InvalidAttributeException();
        }
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
            while (mQueueDpsimToInterface.try_dequeue(receivedPacket)) {

                //TODO: Save the type of the attributes somewhere to perform the conversion
                **(mImportAttrsDpsim[receivedPacket.attributeId]) = **receivedPacket.value;
            }
        }
    }

    void InterfaceManager::pushDpsimAttrsToQueue() {
        for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
            mQueueDpsimToInterface.enqueue(AttributePacket {
                mExportAttrsDpsim[i],
                i,
                mCurrentSequenceDpsimToInterface++
            });
        }
    }
}

