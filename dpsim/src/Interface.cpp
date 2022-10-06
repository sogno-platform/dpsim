// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>

using namespace CPS;

namespace DPsim {

    void Interface::open() {
        mInterfaceWorker->open();
        mInterfaceWriterThread = std::thread(Interface::WriterThread(mQueueDpsimToInterface, mInterfaceWorker));
        mInterfaceReaderThread = std::thread(Interface::ReaderThread(mQueueInterfaceToDpsim, mInterfaceWorker, mOpened));
        mOpened = true;
    }

    void Interface::close() {
	    mOpened = false;
        mQueueDpsimToInterface->enqueue(AttributePacket {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_CLOSE_INTERFACE
        });
        mInterfaceWriterThread.join();
        mInterfaceReaderThread.join();
        mInterfaceWorker->close();
    }

    CPS::Task::List Interface::getTasks() {
        return CPS::Task::List({
            std::make_shared<Interface::PreStep>(*this),
            std::make_shared<Interface::PostStep>(*this)
        });
    }

    void Interface::PreStep::execute(Real time, Int timeStepCount) {
        if (!mIntf.mImportAttrsDpsim.empty()) {
            if (timeStepCount % mIntf.mDownsampling == 0)
                mIntf.popDpsimAttrsFromQueue();
        }	
    }

    void Interface::PostStep::execute(Real time, Int timeStepCount) {
        if (!mIntf.mExportAttrsDpsim.empty()) {
            if (timeStepCount % mIntf.mDownsampling == 0)
                mIntf.pushDpsimAttrsToQueue();
        }
    }

    void Interface::importAttribute(CPS::AttributeBase::Ptr attr, bool blockOnRead) {
        mImportAttrsDpsim.push_back(std::make_tuple(attr, 0, blockOnRead));
        //mInterfaceWorker->configureImport(mImportAttrsDpsim.size() - 1, attr->getType());
    }

    void Interface::exportAttribute(CPS::AttributeBase::Ptr attr) {
        mExportAttrsDpsim.push_back(std::make_tuple(attr, 0));
        //mInterfaceWorker->configureExport(mImportAttrsDpsim.size() - 1, attr->getType());
    }

    void Interface::popDpsimAttrsFromQueue() {
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

    void Interface::pushDpsimAttrsToQueue() {
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

    void Interface::WriterThread::operator() () {
        bool interfaceClosed = false;
        std::vector<Interface::AttributePacket> attrsToWrite;
        while (!interfaceClosed) {
            AttributePacket nextPacket;
            if(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
                if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                    interfaceClosed = true;
                } else {
                    attrsToWrite.push_back(nextPacket);
                    mInterfaceWorker->writeValuesToEnv(attrsToWrite);
                }
            }
            
        }
    }

    void Interface::ReaderThread::operator() () {
        std::vector<Interface::AttributePacket>  attrsRead;
        while (mOpened) { //TODO: As long as reading blocks, there is no real way to force-stop thread execution from the dpsim side
            mInterfaceWorker->readValuesFromEnv(attrsRead);
            for (auto packet : attrsRead) {
                mQueueInterfaceToDpsim->enqueue(packet);
            }
            attrsRead.clear();
        }
    }

}

