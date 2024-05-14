// SPDX-License-Identifier: Apache-2.0

#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim-villas/InterfaceWorkerVillas.h>

using namespace villas;

namespace DPsim {

InterfaceVillas::InterfaceVillas(const String &nodeConfig, UInt queueLength,
                                 UInt sampleLength, const String &name,
                                 UInt downsampling)
    : Interface(
          InterfaceWorkerVillas::make(nodeConfig, queueLength, sampleLength),
          name, downsampling) {}

void InterfaceVillas::importAttribute(CPS::AttributeBase::Ptr attr, UInt idx,
                                      Bool blockOnRead,
                                      Bool syncOnSimulationStart) {
  Interface::addImport(attr, blockOnRead, syncOnSimulationStart);
  std::dynamic_pointer_cast<InterfaceWorkerVillas>(mInterfaceWorker)
      ->configureImport((UInt)mImportAttrsDpsim.size() - 1, attr->getType(),
                        idx);
}

void InterfaceVillas::exportAttribute(CPS::AttributeBase::Ptr attr, UInt idx,
                                      Bool waitForOnWrite, const String &name,
                                      const String &unit) {
  Interface::addExport(attr);
  std::dynamic_pointer_cast<InterfaceWorkerVillas>(mInterfaceWorker)
      ->configureExport((UInt)mExportAttrsDpsim.size() - 1, attr->getType(),
                        idx, waitForOnWrite, name, unit);
}

} // namespace DPsim
