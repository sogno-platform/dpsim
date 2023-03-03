// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/InterfaceWorker.h>

#include <villas/node.hpp>
#include <villas/exceptions.hpp>
#include <villas/memory.hpp>
#include <villas/kernel/rt.hpp>
#include <villas/pool.hpp>
#include <villas/sample.hpp>
#include <villas/signal.hpp>
#include <villas/signal_list.hpp>

namespace DPsim {
	class InterfaceWorkerVillas :
		public InterfaceWorker,
		public SharedFactory<InterfaceWorkerVillas> {

	public:
		using Ptr = std::shared_ptr<InterfaceWorkerVillas>;

		static UInt villasPriority;
		static UInt villasAffinity;
		static UInt villasHugePages;

	private:
		static Bool villasInitialized;

		std::vector<std::tuple<std::function<CPS::AttributeBase::Ptr(villas::node::Sample*)>, UInt>> mImports;
		std::vector<std::tuple<std::function<void(CPS::AttributeBase::Ptr, villas::node::Sample*)>, UInt, Bool>> mExports;

		// VILLASnode node to send / receive data to / from
		String mNodeConfig;
		villas::node::Node* mNode;

		int mQueueLength;
		int mSampleLength;
		villas::node::Pool mSamplePool;

		villas::node::Sample *mLastSample;
		int mSequence;

		std::map<int, villas::node::Signal::Ptr> mExportSignals;
		std::map<int, villas::node::Signal::Ptr> mImportSignals;

	public:
		InterfaceWorkerVillas(const String &nodeConfig, UInt queueLength = 512, UInt sampleLength = 64);

		void open() override;
		void close() override;

		void readValuesFromEnv(std::vector<Interface::AttributePacket>& updatedAttrs) override;
		void writeValuesToEnv(std::vector<Interface::AttributePacket>& updatedAttrs) override;

        virtual void configureImport(UInt attributeId, const std::type_info& type, UInt idx);
        virtual void configureExport(UInt attributeId, const std::type_info& type, UInt idx, Bool waitForOnWrite, const String& name = "", const String& unit = "");

	private:
		void prepareNode();
		void setupNodeSignals();
		void initVillas() const;
	};
}

