#pragma once

#include <atomic>

#include <villas/shmem.h>

#include "ExternalInterface.h"
#include "Components/ExternalCurrentSource.h"
#include "Components/ExternalVoltageSource.h"

namespace DPsim {

	/** \brief Implements ExternalInterface by using the shared memory interface of VILLASnode.
	 *
	 * For this class, the indexes are the offsets in the data member of VILLAS's
	 * struct sample. Make sure that VILLASnode is configured accordingly.
	 */
	class VillasInterface : public ExternalInterface {
	private:
		const char* mShmemName;
		struct shmem_int mShmem;
		int mSeq;

	public:
		/** Create a VillasInterface using the given shmem object name.
		 *
		 * @param name The name of the POSIX shmem object (like given in the
		 * configuration of VILLASnode).
		 */
		VillasInterface(const char* name);
		VillasInterface(const char* name, struct shmem_conf* conf);
		~VillasInterface();
		/** Read a single struct sample from the shared input queue and pass the contained
		 * values to all registered current/voltage sources.
		 */
		virtual void readValues();
		/** Collect all exported currents and voltages in a struct sample and
		 * write it to the shared output queue.
		 */
		virtual void writeValues(SystemModel &model);
	};
};
