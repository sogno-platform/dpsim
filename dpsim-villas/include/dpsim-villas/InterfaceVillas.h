// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/PtrFactory.h>
#include <dpsim/InterfaceManager.h>

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
		typedef std::shared_ptr<InterfaceVillas> Ptr;
		typedef struct node::Sample Sample;

		static UInt villasPriority;
		static UInt villasAffinity;
		static UInt villasHugePages;

	protected:
		static Bool villasInitialized;

		std::vector<std::tuple<std::function<CPS::AttributeBase::Ptr(Sample*)>, UInt>> mImports;
		std::vector<std::tuple<std::function<void(CPS::AttributeBase::Ptr, Sample*)>, UInt, Bool>> mExports;
		
		//Villas node to send / receive data to / from
		String mNodeConfig;
		node::Node* mNode;

		int mQueueLength;
		int mSampleLength;
		node::Pool mSamplePool;

		Sample *mLastSample;
		int mSequence;

		std::map<int, node::Signal::Ptr> mExportSignals;
		std::map<int, node::Signal::Ptr> mImportSignals;

	public:

		InterfaceVillas(const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64);

		virtual void open() override;
		virtual void close() override;
	
		virtual void readValuesFromEnv(std::vector<InterfaceManager::AttributePacket>& updatedAttrs) override;
		virtual void writeValuesToEnv(std::vector<InterfaceManager::AttributePacket>& updatedAttrs) override;

        virtual void configureImport(UInt attributeId, std::type_info type, UInt idx);
        virtual void configureExport(UInt attributeId, std::type_info type, UInt idx, Bool waitForOnWrite, String name = "", String unit = "");
	
	private:
		void prepareNode();
		void setupNodeSignals();
		void initVillas();
	};
}

