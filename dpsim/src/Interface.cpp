// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>

using namespace CPS;

namespace DPsim {

    void Interface::open() {
        mOpened = true;
    }

    void Interface::close() {
        //TODO: Close threads / queue
	    mOpened = false;
    }

    CPS::Task::List Interface::getTasks() {
        //TODO: Should this only be two tasks (reading + writing) or one task for every attribute that is imported / exported?
        //Due to the dependencies on external, it should not matter --> verify this behavior?
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

    void Interface::importAttribute(CPS::AttributeBase::Ptr attr) {
        if (attr->isStatic()) {
            mLog->error("Cannot import to a static attribute. Please provide a dynamic attribute!");
            throw InvalidAttributeException();
        }
        mImportAttrsDpsim.push_back(attr);
    }

    void Interface::exportAttribute(CPS::AttributeBase::Ptr attr) {
        mExportAttrsDpsim.push_back(attr);
    }

    void Interface::popDpsimAttrsFromQueue() {
        //UNIMPLEMENTED!
    }

    void Interface::pushDpsimAttrsToQueue() {
        //UNIMPLEMENTED!
    }

}

