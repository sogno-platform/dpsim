#include <dpsim-models/Logger.h>
#include <dpsim/Config.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>
#include <dpsim-models/Attribute.h>
#include <dpsim-models/Task.h>


namespace DPsim {

	class Interface {

    protected:
        bool mOpened;
        CPS::Logger::Log mLog;
    
    public:
        //Function that will be called on loop in its separate thread.
		//Should be used to read values from the environment and push them into `updatedAttrs`
		virtual void readValuesFromEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

		//Function that will be called on loop in its separate thread.
		//Should be used to read values from `updatedAttrs` and write them to the environment
		virtual void writeValuesToEnv(CPS::AttributeBase::List& updatedAttrs) = 0;

        virtual void open(CPS::Logger::Log log);
        virtual void close();

    }
}