#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>


namespace DPsim {

    class InterfaceManager;

	class Interface {

    protected:
        bool mOpened;
        CPS::Logger::Log mLog;
        std::weak_ptr<InterfaceManager> mManager;
    
    public:
        typedef std::shared_ptr<Interface> Ptr;

        /**
         * Function that will be called on loop in its separate thread.
         * Should be used to read values from the environment and push them into `updatedAttrs`
         * `updatedAttrs` will always be empty when this function is invoked
         */
		virtual void readValuesFromEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

		/**
		 * Function that will be called on loop in its separate thread.
         * Should be used to read values from `updatedAttrs` and write them to the environment
         * Every new value will only be passed in `updatedAttrs` once, so the interface needs to buffer these values if they are not send right away
		 */
        virtual void writeValuesToEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

        /**
         * Open the interface and set up the connection to the environment
         * This is guaranteed to be called before any calls to `readValuesFromEnv` and `writeValuesToEnv`
         */
        virtual void open();

        /**
         * Close the interface and all connections to the environment
         * After this has been called, no further calls to `readValuesFromEnv` or `writeValuesToEnv` will occur
         */
        virtual void close();

        /**
         * Will try to get a pointer to the parent `InterfaceManager` instance
         * Will throw an error and exit if this interface has no manager
         */
        std::shared_ptr<InterfaceManager> tryGetManager() {
            std::shared_ptr<InterfaceManager> manager = mManager.lock();
            if (manager == nullptr) {
                mLog->error("Error: The interface is not connected yet to a manager! Please add this interface to a simulation first before configuring imports and exports.");
                std::exit(1);
            }
            return manager;
        }

    };
}