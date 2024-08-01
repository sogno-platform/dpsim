// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/InterfaceQueued.h>
#include <dpsim/InterfaceWorker.h>

#include <spdlog/common.h>
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
class InterfaceWorkerVillas : public InterfaceWorker,
                              public SharedFactory<InterfaceWorkerVillas> {

public:
  using Ptr = std::shared_ptr<InterfaceWorkerVillas>;
  using Sample = struct node::Sample;

  static UInt villasPriority;
  static UInt villasAffinity;
  static UInt villasHugePages;

private:
  static Bool villasInitialized;

  std::vector<
      std::tuple<std::function<CPS::AttributeBase::Ptr(Sample *)>, UInt>>
      mImports;
  std::vector<std::tuple<std::function<void(CPS::AttributeBase::Ptr, Sample *)>,
                         UInt, Bool>>
      mExports;

  // VILLASnode node to send / receive data to / from
  String mNodeConfig;
  node::Node *mNode;

  int mQueueLength;
  int mSampleLength;
  node::Pool mSamplePool;

  Sample *mLastSample;
  int mSequence;

  std::map<int, node::Signal::Ptr> mExportSignals;
  std::map<int, node::Signal::Ptr> mImportSignals;

public:
  InterfaceWorkerVillas(
      const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64,
      spdlog::level::level_enum logLevel = spdlog::level::level_enum::info);

  void open() override;
  void close() override;

  void readValuesFromEnv(
      std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) override;
  void readValuesFromEnv(
      std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt, bool, bool>>
          &updatedAttrs);
  void writeValuesToEnv(
      std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) override;
  void writeValuesToEnv(
      std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt>> &updatedAttrs);

  virtual void configureImport(UInt attributeId, const std::type_info &type,
                               UInt idx, const String &name = "",
                               const String &unit = "");
  virtual void configureExport(UInt attributeId, const std::type_info &type,
                               UInt idx, Bool waitForOnWrite,
                               const String &name = "",
                               const String &unit = "");

  void printSignals() const;

private:
  void prepareNode();
  void setupNodeSignals();
  void initVillas() const;
};
} // namespace DPsim
