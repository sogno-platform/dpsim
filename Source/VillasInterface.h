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
		struct shmem_int mShmem;
		int mSeq;

	public:
		/** Create a VillasInterface using the given shmem object names.
		 *
		 * @param wname The name of the POSIX shmem object (like given in the
		 * configuration of VILLASnode) were samples will be written to.
		 * @param rname The name of the POSIX shmem object were samples will be read from.
		 */
		VillasInterface(const char* wname, const char* rname);
		/** Create a VillasInterface with a specific configuration for the output queue.
		 *
		 * @param conf The configuration object for the output queue (see VILLASnode's documentation).
		 */
		VillasInterface(const char* wname, const char* rname, struct shmem_conf* conf);
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
