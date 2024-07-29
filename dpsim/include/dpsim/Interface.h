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

namespace DPsim {

class InterfaceWorker;

class Interface : public SharedFactory<Interface> {

public:
  typedef std::shared_ptr<Interface> Ptr;

  Interface(const String &name = "", spdlog::level::level_enum logLevel =
                                         spdlog::level::level_enum::info)
      : mName(name), mOpened(false) {
    mLog = CPS::Logger::get("Interface", logLevel);
  };

  virtual void open() = 0;
  virtual void close() = 0;

  // Function called by the Simulation to perform interface synchronization
  virtual void syncExports() = 0;
  /// Function called by the Simulation to perform interface synchronization
  virtual void syncImports() = 0;

  virtual CPS::Task::List getTasks() = 0;

  virtual void setLogger(CPS::Logger::Log log);

  virtual String &getName() { return mName; }

  // Attributes used in the DPsim simulation. Should only be accessed by the dpsim-thread
  // Tuple attributes: Attribute to be imported, Current sequenceID, blockOnRead, syncOnSimulationStart
  std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt, bool, bool>>
      mImportAttrsDpsim;
  // Tuple attributes: Attribute to be exported, Current Sequence ID
  std::vector<std::tuple<CPS::AttributeBase::Ptr, UInt>> mExportAttrsDpsim;

  virtual void addImport(CPS::AttributeBase::Ptr attr, bool blockOnRead = false,
                         bool syncOnSimulationStart = true);
  virtual void addExport(CPS::AttributeBase::Ptr attr);

protected:
  CPS::Logger::Log mLog;
  String mName;
  bool mSyncOnSimulationStart;
  UInt mCurrentSequenceDpsimToInterface = 1;
  UInt mNextSequenceInterfaceToDpsim = 1;
  std::atomic<bool> mOpened;
};
} // namespace DPsim
