/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

// #############################################
// Do NOT include this header in any MPL2 files
// #############################################

#pragma once

#include <dpsim-villas/InterfaceSampleBased.h>

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

