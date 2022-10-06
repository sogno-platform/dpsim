// SPDX-License-Identifier: Apache-2.0

#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim-villas/InterfaceWorkerVillas.h>

using namespace villas;

namespace DPsim {

    InterfaceVillas::InterfaceVillas(const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64, String name = "", bool syncOnSimulationStart = false, UInt downsampling = 1)
        : Interface(InterfaceWorkerVillas::make(nodeConfig, queueLenght, sampleLenght), name, syncOnSimulationStart, downsampling) { }

    void InterfaceVillas::importAttribute(CPS::AttributeBase::Ptr attr, UInt idx, Bool blockOnRead) {
        Interface::importAttribute(attr, blockOnRead);
        std::dynamic_pointer_cast<InterfaceWorkerVillas>(mInterfaceWorker)->configureImport(mImportAttrsDpsim.size() - 1, attr->getType(), idx);
    }

	void InterfaceVillas::exportAttribute(CPS::AttributeBase::Ptr attr, UInt idx, Bool waitForOnWrite, String name, String unit) {
        Interface::exportAttribute(attr);
        std::dynamic_pointer_cast<InterfaceWorkerVillas>(mInterfaceWorker)->configureExport(mImportAttrsDpsim.size() - 1, attr->getType(), idx, waitForOnWrite, name, unit);
    }

}