// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim/Villas/InterfaceSampleBased.h>

#include <villas/shmem.hpp>

using namespace villas;

namespace DPsim {
	/// Shmem interface used in combination with VILLAS
	class InterfaceShmem :
		public InterfaceSampleBased,
		public SharedFactory<InterfaceShmem> {

	public:
		typedef std::shared_ptr<InterfaceShmem> Ptr;
		typedef struct node::ShmemConfig Config;
		typedef struct node::ShmemInterface ShmemInt;

	protected:
		ShmemInt mShmem;
		Config mConf;

	public:
		/** Create a InterfaceShmem with a specific configuration for the output queue.
		 *
		 * @param wname The name of the POSIX shmem object where samples will be written to.
		 * @param rname The name of the POSIX shmem object where samples will be read from.
		 * @param conf The configuration object for the output queue (see VILLASnode's documentation), or nullptr for sensible defaults.
		 */
		InterfaceShmem(const String &wn, const String &rn, Config *conf = nullptr, Bool sync = true, UInt downsampling = 1) :
			InterfaceSampleBased(wn, rn, sync, downsampling)
		{
			if (conf != nullptr) {
				mConf = *conf;
			} else {
				mConf.queuelen = 512;
				mConf.samplelen = 64;
				mConf.polling = 0;
			}
		}

		void open(CPS::Logger::Log log);
		void close();

		void readValues(bool blocking = true);
		void writeValues();
	};
}

