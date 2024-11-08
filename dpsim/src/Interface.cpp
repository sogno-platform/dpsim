// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>

using namespace CPS;

namespace DPsim {

void Interface::addImport(CPS::AttributeBase::Ptr attr, bool blockOnRead,
                          bool syncOnSimulationStart) {
  if (mOpened) {
    SPDLOG_LOGGER_ERROR(
        mLog, "Cannot modify interface configuration after simulation start!");
    std::exit(1);
  }

  mImportAttrsDpsim.emplace_back(attr, 0, blockOnRead, syncOnSimulationStart);
}

void Interface::addExport(CPS::AttributeBase::Ptr attr) {
  if (mOpened) {
    SPDLOG_LOGGER_ERROR(
        mLog, "Cannot modify interface configuration after simulation start!");
    std::exit(1);
  }

  mExportAttrsDpsim.emplace_back(attr, 0);
}

void Interface::setLogger(CPS::Logger::Log log) { mLog = log; }

} // namespace DPsim
