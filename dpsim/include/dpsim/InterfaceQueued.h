// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <thread>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/Task.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Interface.h>
#include <dpsim/Scheduler.h>

#include <readerwriterqueue.h>

namespace DPsim {

class InterfaceWorker;

class InterfaceQueued : public Interface,
                        public SharedFactory<InterfaceQueued> {

public:
  typedef std::shared_ptr<InterfaceQueued> Ptr;

  using AttributePacket = struct AttributePacket {
    CPS::AttributeBase::Ptr value;
    UInt
        attributeId; //Used to identify the attribute. Defined by the position in the `mExportAttrsDpsim` and `mImportAttrsDpsim` lists
    UInt
        sequenceId; //Increasing ID used to discern multiple consecutive updates of a single attribute
    unsigned char flags; //Bit 0 set: Close interface
  };

  enum AttributePacketFlags {
    PACKET_NO_FLAGS = 0,
    PACKET_CLOSE_INTERFACE = 1,
  };

  InterfaceQueued(std::shared_ptr<InterfaceWorker> intf,
                  const String &name = "", UInt downsampling = 1)
      : Interface(name), mInterfaceWorker(intf), mDownsampling(downsampling) {
    mQueueDpsimToInterface = std::make_shared<
        moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
    mQueueInterfaceToDpsim = std::make_shared<
        moodycamel::BlockingReaderWriterQueue<AttributePacket>>();
  };

  virtual void open() override;
  virtual void close() override;

  // Function used in the interface's simulation task to read all imported attributes from the queue
  // Called once before every simulation timestep
  virtual void pushDpsimAttrsToQueue();
  // Function used in the interface's simulation task to write all exported attributes to the queue
  // Called once after every simulation timestep
  virtual void popDpsimAttrsFromQueue(bool isSync = false);

  // Function called by the Simulation to perform interface synchronization
  virtual void syncExports() override;
  /// Function called by the Simulation to perform interface synchronization
  virtual void syncImports() override;

  virtual CPS::Task::List getTasks() override;

  virtual void setLogger(CPS::Logger::Log log) override;

  virtual ~InterfaceQueued() {
    if (mOpened)
      close();
  }

protected:
  std::shared_ptr<InterfaceWorker> mInterfaceWorker;
  UInt mDownsampling;
  std::thread mInterfaceWriterThread;
  std::thread mInterfaceReaderThread;

  std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
      mQueueDpsimToInterface;
  std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
      mQueueInterfaceToDpsim;

public:
  class WriterThread {
  private:
    std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
        mQueueDpsimToInterface;
    std::shared_ptr<InterfaceWorker> mInterfaceWorker;

  public:
    WriterThread(
        std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
            queueDpsimToInterface,
        std::shared_ptr<InterfaceWorker> intf)
        : mQueueDpsimToInterface(queueDpsimToInterface),
          mInterfaceWorker(intf){};
    void operator()() const;
  };

  class ReaderThread {
  private:
    std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
        mQueueInterfaceToDpsim;
    std::shared_ptr<InterfaceWorker> mInterfaceWorker;
    std::atomic<bool> &mOpened;

  public:
    ReaderThread(
        std::shared_ptr<moodycamel::BlockingReaderWriterQueue<AttributePacket>>
            queueInterfaceToDpsim,
        std::shared_ptr<InterfaceWorker> intf, std::atomic<bool> &opened)
        : mQueueInterfaceToDpsim(queueInterfaceToDpsim), mInterfaceWorker(intf),
          mOpened(opened){};
    void operator()() const;
  };

  class PreStep : public CPS::Task {
  public:
    explicit PreStep(InterfaceQueued &intf)
        : Task(intf.getName() + ".Read"), mIntf(intf) {
      for (const auto &[attr, _seqId, _blockOnRead, _syncOnStart] :
           intf.mImportAttrsDpsim) {
        mModifiedAttributes.push_back(attr);
      }
    }

    void execute(Real time, Int timeStepCount) override;

  private:
    InterfaceQueued &mIntf;
  };

  class PostStep : public CPS::Task {
  public:
    explicit PostStep(InterfaceQueued &intf)
        : Task(intf.getName() + ".Write"), mIntf(intf) {
      for (const auto &[attr, _seqId] : intf.mExportAttrsDpsim) {
        mAttributeDependencies.push_back(attr);
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount) override;

  private:
    InterfaceQueued &mIntf;
  };
};
} // namespace DPsim
