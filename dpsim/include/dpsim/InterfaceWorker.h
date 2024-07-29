// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <typeinfo>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/Task.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Interface.h>
#include <dpsim/InterfaceQueued.h>
#include <dpsim/Scheduler.h>

namespace DPsim {

class InterfaceWorker {

protected:
  bool mOpened;
  UInt mCurrentSequenceInterfaceToDpsim = 1;

public:
  using Ptr = std::shared_ptr<InterfaceWorker>;

  CPS::Logger::Log mLog;

  InterfaceWorker() = default;
  virtual ~InterfaceWorker() = default;

  /**
         * Function that will be called on loop in its separate thread.
         * Should be used to read values from the environment and push them into `updatedAttrs`
         * `updatedAttrs` will always be empty when this function is invoked
         */
  virtual void readValuesFromEnv(
      std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) = 0;

  /**
		 * Function that will be called on loop in its separate thread.
         * Should be used to read values from `updatedAttrs` and write them to the environment
         * The `updatedAttrs` list will not be cleared by the caller in between function calls
         * When this function is called, `updatedAttrs` will include at least one value
		 */
  virtual void writeValuesToEnv(
      std::vector<InterfaceQueued::AttributePacket> &updatedAttrs) = 0;

  /**
         * Open the interface and set up the connection to the environment
         * This is guaranteed to be called before any calls to `readValuesFromEnv` and `writeValuesToEnv`
         */
  virtual void open() = 0;

  /**
         * Close the interface and all connections to the environment
         * After this has been called, no further calls to `readValuesFromEnv` or `writeValuesToEnv` will occur
         */
  virtual void close() = 0;
};
} // namespace DPsim
