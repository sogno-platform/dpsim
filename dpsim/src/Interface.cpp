// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>

using namespace CPS;

namespace DPsim {

    void Interface::importAttribute(CPS::AttributeBase::Ptr attr, UInt idx) {
        if (attr->isStatic()) {
            //TODO: Give the interface the option to log
            //mLog->error("Cannot import to a static attribute. Please provide a dynamic attribute!");
            throw InvalidAttributeException();
        }
        if (auto attrReal = std::dynamic_pointer_cast<CPS::Attribute<Real>>(attr.getPtr())) {
            attrReal->setReference(importReal(idx));
        } else if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<Complex>>(attr.getPtr())) {
            attrComp->setReference(importComplex(idx));
        } else if (auto attrInt = std::dynamic_pointer_cast<CPS::Attribute<Int>>(attr.getPtr())) {
            attrInt->setReference(importInt(idx));
        } else if (auto attrBool = std::dynamic_pointer_cast<CPS::Attribute<Bool>>(attr.getPtr())) {
            attrBool->setReference(importBool(idx));
        } else {
            //TODO: Give the interface the option to log
            //mLog->error("Only scalar attributes of type Int, Bool, Real or Complex can be imported.");
            throw InvalidAttributeException();
        }
    }

    void Interface::exportAttribute(CPS::AttributeBase::Ptr attr, Int idx) {
        if (auto attrReal = std::dynamic_pointer_cast<CPS::Attribute<Real>>(attr.getPtr())) {
            exportReal(attrReal, idx);
        } else if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<Complex>>(attr.getPtr())) {
            exportComplex(attrComp, idx);
        } else if (auto attrInt = std::dynamic_pointer_cast<CPS::Attribute<Int>>(attr.getPtr())) {
            exportInt(attrInt, idx);
        } else if (auto attrBool = std::dynamic_pointer_cast<CPS::Attribute<Bool>>(attr.getPtr())) {
            exportBool(attrBool, idx);
        } else {
            //TODO: Give the interface the option to log
            //mLog->error("Only scalar attributes of type Int, Bool, Real or Complex can be exported. Use the Attribute::derive methods to export individual Matrix coefficients!");
        }
    }

}

