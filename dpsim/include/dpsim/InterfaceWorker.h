// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <typeinfo>

#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>

namespace DPsim {

    class Interface;

	class InterfaceWorker {

    protected:
        bool mOpened;
        UInt mCurrentSequenceInterfaceToDpsim = 0;
    
    public:
        typedef std::shared_ptr<Interface> Ptr;

        CPS::Logger::Log mLog;
        std::weak_ptr<Interface> mManager;

        InterfaceWorker() = default;
        virtual ~InterfaceWorker() { }

        /**
         * Function that will be called on loop in its separate thread.
         * Should be used to read values from the environment and push them into `updatedAttrs`
         * `updatedAttrs` will always be empty when this function is invoked
         */
		virtual void readValuesFromEnv(std::vector<Interface::AttributePacket>& updatedAttrs) = 0;

		/**
		 * Function that will be called on loop in its separate thread.
         * Should be used to read values from `updatedAttrs` and write them to the environment
         * The `updatedAttrs` list will not be cleared by the caller in between function calls
		 */
        virtual void writeValuesToEnv(std::vector<Interface::AttributePacket>& updatedAttrs) = 0;

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

        /**
         * Will try to get a pointer to the parent `Interface` instance
         * Will throw an error and exit if this interface has no manager
         */
        std::shared_ptr<Interface> tryGetManager() {
            std::shared_ptr<Interface> manager = mManager.lock();
            if (manager == nullptr) {
                mLog->error("Error: The interface is not connected yet to a manager! Please add this interface to a simulation first before configuring imports and exports.");
                std::exit(1);
            }
            return manager;
        }

    };
}