// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-villas/InterfaceSampleBased.h>

#include <villas/node.hpp>
#include <villas/exceptions.hpp>
#include <villas/memory.hpp>
#include <villas/kernel/rt.hpp>
#include <villas/pool.hpp>

using namespace villas;

namespace DPsim {
	class InterfaceVillas :
		public InterfaceSampleBased,
		public SharedFactory<InterfaceVillas> {

	public:
		typedef std::shared_ptr<InterfaceVillas> Ptr;
		static UInt villasPriority;
		static UInt villasAffinity;
		static UInt villasHugePages;

	protected:
		//Villas node to send / receive data to / from
		String mNodeConfig;
		node::Node* mNode;

		int mQueueLength;
		int mSampleLength;
		node::Pool mSamplePool;

		static Bool villasInitialized;

	public:
		/** Create a InterfaceVillas with a specific configuration for the VillasNode
		 *
		 * @param name The name of the newly created VillasNode
		 */
		InterfaceVillas(const String &name, const String &nodeConfig, UInt queueLenght = 512, UInt sampleLenght = 64, UInt downsampling = 1);

		void open(CPS::Logger::Log log);
		void close();

		void readValues(bool blocking = true);
		void writeValues();
		void initVillas();

	private:
		void prepareNode();
		void setupNodeSignals();
	};
}

