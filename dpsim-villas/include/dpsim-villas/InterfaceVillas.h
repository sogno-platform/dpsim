// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/Interface.h>

#include <villas/node.hpp>
#include <villas/exceptions.hpp>
#include <villas/memory.hpp>
#include <villas/kernel/rt.hpp>
#include <villas/pool.hpp>
#include <villas/sample.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

using namespace villas;

namespace DPsim {
	class InterfaceVillas :
		public Interface,
		public SharedFactory<InterfaceVillas> {

	public:
		InterfaceVillas(const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64, const String& name = "", bool syncOnSimulationStart = false, UInt downsampling = 1);

        void importAttribute(CPS::AttributeBase::Ptr attr, UInt idx, Bool blockOnRead = false);
		void exportAttribute(CPS::AttributeBase::Ptr attr, UInt idx, Bool waitForOnWrite, const String& name = "", const String& unit = "");

	};
}

