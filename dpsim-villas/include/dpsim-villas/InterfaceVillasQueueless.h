// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/Task.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Interface.h>
#include <dpsim/Scheduler.h>

#include <memory>
#include <villas/kernel/rt.hpp>
#include <villas/node.hpp>
#include <villas/node/exceptions.hpp>
#include <villas/node/memory.hpp>
#include <villas/pool.hpp>
#include <villas/sample.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

using namespace villas;

namespace DPsim {
/// Interface type that can be used to import and export simulation attributes over any node type supported by VILLASnode
class InterfaceVillasQueueless
    : public Interface,
      public SharedFactory<InterfaceVillasQueueless> {

public:
  typedef std::shared_ptr<InterfaceVillasQueueless> Ptr;

  /// @brief create a new InterfaceVillasQueueless instance
  /// @param nodeConfig VILLASnode node configuration in JSON format
  /// @param name Name of this interface. Currently only used for naming the simulation tasks
  InterfaceVillasQueueless(
      const String &nodeConfig, const String &name = "",
      spdlog::level::level_enum logLevel = spdlog::level::level_enum::info);

  virtual void open() override;
  virtual void close() override;

  // Function called by the Simulation to perform interface synchronization
  virtual void syncExports() override;
  /// Function called by the Simulation to perform interface synchronization
  virtual void syncImports() override;

  virtual CPS::Task::List getTasks() override;

  virtual void printVillasSignals() const;

  virtual ~InterfaceVillasQueueless() {
    if (mOpened)
      close();
  }

protected:
  const String mNodeConfig;
  node::Node *mNode;
  node::Pool mSamplePool;

  virtual void writeToVillas();
  virtual Int readFromVillas();
  void createNode();
  void createSignals();
  Int mSequenceToDpsim;
  Int mSequenceFromDpsim;

public:
  class PreStep : public CPS::Task {
  public:
    explicit PreStep(InterfaceVillasQueueless &intf)
        : Task(intf.mName + ".Read"), mIntf(intf) {
      for (const auto &[attr, _seqId, _blockOnRead, _syncOnStart] :
           intf.mImportAttrsDpsim) {
        mModifiedAttributes.push_back(attr);
      }
    }

    void execute(Real time, Int timeStepCount) override;

  private:
    InterfaceVillasQueueless &mIntf;
  };

  class PostStep : public CPS::Task {
  public:
    explicit PostStep(InterfaceVillasQueueless &intf)
        : Task(intf.mName + ".Write"), mIntf(intf) {
      for (const auto &[attr, _seqId] : intf.mExportAttrsDpsim) {
        mAttributeDependencies.push_back(attr);
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount) override;

  private:
    InterfaceVillasQueueless &mIntf;
  };
};
} // namespace DPsim
