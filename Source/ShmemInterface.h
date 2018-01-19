/** Shared-memory interface
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include <atomic>

#include <villas/shmem.h>

#include "ExternalInterface.h"

namespace DPsim {

	/** \brief Implements ExternalInterface by using the shared memory interface of VILLASnode.
	 *
	 * For this class, the indexes are the offsets in the data member of VILLAS's
	 * struct sample. Make sure that VILLASnode is configured accordingly.
	 */
	class ShmemInterface : public ExternalInterface {

	private:
		struct shmem_int mShmem;
		struct sample *mLastSample;
		int mSeq;
		std::string rname, wname;
		int mInit;

		void init(const String &wname, const String &rname, struct shmem_conf *conf);

	public:
		/** Create a ShmemInterface using the given shmem object names.
		 *
		 * @param wname The name of the POSIX shmem object where samples will be written to.
		 * @param rname The name of the POSIX shmem object where samples will be read from.
		 */
		ShmemInterface(const String &wname, const String &rname);

		/** Create a ShmemInterface with a specific configuration for the output queue.
		 *
		 * @param conf The configuration object for the output queue (see VILLASnode's documentation).
		 */
		ShmemInterface(const String &wname, const String &rname, struct shmem_conf* conf);

		~ShmemInterface();

		/** Read a single struct sample from the shared input queue and pass the contained
		 * values to all registered current/voltage sources.
		 */
		virtual void readValues(bool blocking = true);

		/** Collect all exported currents and voltages in a struct sample and
		 * write it to the shared output queue.
		 */
		virtual void writeValues(SystemModel &model);
	};
};

