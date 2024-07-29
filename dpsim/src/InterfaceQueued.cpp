// SPDX-License-Identifier: Apache-2.0

#include <dpsim/InterfaceQueued.h>
#include <dpsim/InterfaceWorker.h>

using namespace CPS;

namespace DPsim {

void InterfaceQueued::open() {
  mInterfaceWorker->open();
  mOpened = true;

  if (!mImportAttrsDpsim.empty()) {
    mInterfaceReaderThread = std::thread(InterfaceQueued::ReaderThread(
        mQueueInterfaceToDpsim, mInterfaceWorker, mOpened));
  }
  if (!mExportAttrsDpsim.empty()) {
    mInterfaceWriterThread = std::thread(InterfaceQueued::WriterThread(
        mQueueDpsimToInterface, mInterfaceWorker));
  }
}

void InterfaceQueued::close() {
  mOpened = false;
  mQueueDpsimToInterface->emplace(AttributePacket{
      nullptr, 0, 0, AttributePacketFlags::PACKET_CLOSE_INTERFACE});

  if (!mExportAttrsDpsim.empty()) {
    mInterfaceWriterThread.join();
  }

  if (!mImportAttrsDpsim.empty()) {
    mInterfaceReaderThread.join();
  }
  mInterfaceWorker->close();
}

CPS::Task::List InterfaceQueued::getTasks() {
  auto tasks = CPS::Task::List();
  if (!mImportAttrsDpsim.empty()) {
    tasks.push_back(std::make_shared<InterfaceQueued::PreStep>(*this));
  }
  if (!mExportAttrsDpsim.empty()) {
    tasks.push_back(std::make_shared<InterfaceQueued::PostStep>(*this));
  }
  return tasks;
}

void InterfaceQueued::PreStep::execute(Real time, Int timeStepCount) {
  if (timeStepCount % mIntf.mDownsampling == 0)
    mIntf.popDpsimAttrsFromQueue();
}

void InterfaceQueued::PostStep::execute(Real time, Int timeStepCount) {
  if (timeStepCount % mIntf.mDownsampling == 0)
    mIntf.pushDpsimAttrsToQueue();
}

void InterfaceQueued::setLogger(CPS::Logger::Log log) {
  Interface::setLogger(log);
  if (mInterfaceWorker != nullptr) {
    mInterfaceWorker->mLog = log;
  }
}

void InterfaceQueued::syncImports() {
  //Block on read until all attributes with syncOnSimulationStart are read
  this->popDpsimAttrsFromQueue(true);
}

void InterfaceQueued::syncExports() {
  //Just push all the attributes
  this->pushDpsimAttrsToQueue();
}

void InterfaceQueued::popDpsimAttrsFromQueue(bool isSync) {
  AttributePacket receivedPacket = {nullptr, 0, 0,
                                    AttributePacketFlags::PACKET_NO_FLAGS};
  UInt currentSequenceId = mNextSequenceInterfaceToDpsim;

  //Wait for and dequeue all attributes that read should block on
  //The std::find_if will look for all attributes that have not been updated in the current while loop (i. e. whose sequence ID is lower than the next expected sequence ID)
  while (std::find_if(mImportAttrsDpsim.cbegin(), mImportAttrsDpsim.cend(),
                      [currentSequenceId, isSync](auto attrTuple) {
                        auto &[_attr, seqId, blockOnRead, syncOnStart] =
                            attrTuple;
                        if (isSync) {
                          return syncOnStart && seqId < currentSequenceId;
                        } else {
                          return blockOnRead && seqId < currentSequenceId;
                        }
                      }) != mImportAttrsDpsim.cend()) {
    if (mQueueInterfaceToDpsim->try_dequeue(receivedPacket) != false) {
      int i = 0;
      while (mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
        i++;
      }
      SPDLOG_LOGGER_WARN(mLog,
                         "Overrun detected! Discarding {} overrun packets!", i);
    } else {
      mQueueInterfaceToDpsim->wait_dequeue(receivedPacket);
    }
    if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])
             ->copyValue(receivedPacket.value)) {
      SPDLOG_LOGGER_WARN(
          mLog, "Failed to copy received value onto attribute in Interface!");
    }
    std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) =
        receivedPacket.sequenceId;
    mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
  }

  //Fetch all remaining queue packets
  while (mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
    if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])
             ->copyValue(receivedPacket.value)) {
      SPDLOG_LOGGER_WARN(
          mLog, "Failed to copy received value onto attribute in Interface!");
    }
    std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) =
        receivedPacket.sequenceId;
    mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
  }
}

void InterfaceQueued::pushDpsimAttrsToQueue() {
  for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
    mQueueDpsimToInterface->emplace(AttributePacket{
        std::get<0>(mExportAttrsDpsim[i])->cloneValueOntoNewAttribute(), i,
        std::get<1>(mExportAttrsDpsim[i]),
        AttributePacketFlags::PACKET_NO_FLAGS});
    std::get<1>(mExportAttrsDpsim[i]) = mCurrentSequenceDpsimToInterface;
    mCurrentSequenceDpsimToInterface++;
  }
}

void InterfaceQueued::WriterThread::operator()() const {
  bool interfaceClosed = false;
  std::vector<InterfaceQueued::AttributePacket> attrsToWrite;
  while (!interfaceClosed) {
    AttributePacket nextPacket = {nullptr, 0, 0,
                                  AttributePacketFlags::PACKET_NO_FLAGS};

    //Wait for at least one packet
    mQueueDpsimToInterface->wait_dequeue(nextPacket);
    if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
      interfaceClosed = true;
    } else {
      attrsToWrite.push_back(nextPacket);
    }

    //See if there are more packets
    while (mQueueDpsimToInterface->try_dequeue(nextPacket)) {
      if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
        interfaceClosed = true;
      } else {
        attrsToWrite.push_back(nextPacket);
      }
    }
    mInterfaceWorker->writeValuesToEnv(attrsToWrite);
  }
}

void InterfaceQueued::ReaderThread::operator()() const {
  std::vector<InterfaceQueued::AttributePacket> attrsRead;
  while (mOpened) {
    mInterfaceWorker->readValuesFromEnv(attrsRead);
    for (const auto &packet : attrsRead) {
      mQueueInterfaceToDpsim->enqueue(packet);
    }
    attrsRead.clear();
  }
}

} // namespace DPsim
