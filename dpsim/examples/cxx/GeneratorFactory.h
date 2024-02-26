#include <DPsim.h>

#pragma once

using namespace DPsim;
using namespace CPS;

namespace GeneratorFactory {

std::shared_ptr<SP::Ph1::ReducedOrderSynchronGeneratorVBR>
createGenSP(const String &SGModel, const String &name, Logger::Level logLevel) {
  std::shared_ptr<SP::Ph1::ReducedOrderSynchronGeneratorVBR> genSP = nullptr;
  if (SGModel == "3")
    genSP = SP::Ph1::SynchronGenerator3OrderVBR::make(name, logLevel);
  else if (SGModel == "4")
    genSP = SP::Ph1::SynchronGenerator4OrderVBR::make(name, logLevel);
  else if (SGModel == "6a")
    genSP = SP::Ph1::SynchronGenerator6aOrderVBR::make(name, logLevel);
  else if (SGModel == "6b")
    genSP = SP::Ph1::SynchronGenerator6bOrderVBR::make(name, logLevel);
  else
    throw SystemError("Unsupported reduced-order SG type!");

  return genSP;
}

std::shared_ptr<DP::Ph1::ReducedOrderSynchronGeneratorVBR>
createGenDP(const String &SGModel, const String &name, Logger::Level logLevel) {
  std::shared_ptr<DP::Ph1::ReducedOrderSynchronGeneratorVBR> genDP = nullptr;
  if (SGModel == "3")
    genDP = DP::Ph1::SynchronGenerator3OrderVBR::make(name, logLevel);
  else if (SGModel == "4")
    genDP = DP::Ph1::SynchronGenerator4OrderVBR::make(name, logLevel);
  else if (SGModel == "6a")
    genDP = DP::Ph1::SynchronGenerator6aOrderVBR::make(name, logLevel);
  else if (SGModel == "6b")
    genDP = DP::Ph1::SynchronGenerator6bOrderVBR::make(name, logLevel);
  else
    throw SystemError("Unsupported reduced-order SG type!");

  return genDP;
}

std::shared_ptr<EMT::Ph3::ReducedOrderSynchronGeneratorVBR>
createGenEMT(const String &SGModel, const String &name,
             Logger::Level logLevel) {
  std::shared_ptr<EMT::Ph3::ReducedOrderSynchronGeneratorVBR> genEMT = nullptr;
  if (SGModel == "3")
    genEMT = EMT::Ph3::SynchronGenerator3OrderVBR::make(name, logLevel);
  else if (SGModel == "4")
    genEMT = EMT::Ph3::SynchronGenerator4OrderVBR::make(name, logLevel);
  else if (SGModel == "6a")
    genEMT = EMT::Ph3::SynchronGenerator6aOrderVBR::make(name, logLevel);
  else if (SGModel == "6b")
    genEMT = EMT::Ph3::SynchronGenerator6bOrderVBR::make(name, logLevel);
  else
    throw SystemError("Unsupported reduced-order SG type!");

  return genEMT;
}

} // namespace GeneratorFactory