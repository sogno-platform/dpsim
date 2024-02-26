// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/Interface.h>

#include <villas/exceptions.hpp>
#include <villas/kernel/rt.hpp>
#include <villas/memory.hpp>
#include <villas/node.hpp>
#include <villas/pool.hpp>
#include <villas/sample.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

using namespace villas;

namespace DPsim {
/// Interface type that can be used to import and export simulation attributes over any node type supported by VILLASnode
class InterfaceVillas : public Interface,
                        public SharedFactory<InterfaceVillas> {

public:
  /// @brief create a new InterfaceVillas instance
  /// @param nodeConfig VILLASnode node configuration in JSON format
  /// @param queueLength queue lenght configured for the node
  /// @param sampleLength sample length configured for the node
  /// @param name Name of this interface. Currently only used for naming the simulation tasks
  /// @param downsampling Only import and export attributes on every nth timestep
  InterfaceVillas(const String &nodeConfig, UInt queueLength = 512,
                  UInt sampleLength = 64, const String &name = "",
                  UInt downsampling = 1);

  /// @brief configure an attribute import
  /// @param attr the attribute that should be updated with the imported values
  /// @param idx The id given to the attribute within VILLASnode samples
  /// @param blockOnRead Whether the simulation should block on every import until the attribute has been updated
  /// @param syncOnSimulationStart Whether the simulation should block before the first timestep until this attribute has been updated
  void importAttribute(CPS::AttributeBase::Ptr attr, UInt idx,
                       Bool blockOnRead = false,
                       Bool syncOnSimulationStart = true);

  /// @brief configure an attribute export
  /// @param attr the attribute which's value should be exported
  /// @param idx The id given to the attribute within VILLASnode samples
  /// @param waitForOnWrite Whether a sample that is sent from this interface is required to contain an updated value of this attribute
  /// @param name Name given to the attribute within VILLASnode samples
  /// @param unit Unit given to the attribute within VILLASnode samples
  void exportAttribute(CPS::AttributeBase::Ptr attr, UInt idx,
                       Bool waitForOnWrite, const String &name = "",
                       const String &unit = "");
};
} // namespace DPsim
