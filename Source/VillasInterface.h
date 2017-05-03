#pragma once

#include <atomic>

#include <villas/shmem.h>

#include "ExternalInterface.h"
#include "Components/ExternalCurrentSource.h"
#include "Components/ExternalVoltageSource.h"

namespace DPsim {
	/** Implements ExternalInterface by using the shared memory interface of VILLASnode. */
	class VillasInterface : public ExternalInterface {
	private:
		const char* mShmemName;
		struct shmem_shared* mShmem;
		int mSeq;
		void* mBase;

	public:
		VillasInterface(const char* name);
		~VillasInterface();
		virtual void readValues();
		virtual void writeValues(SystemModel &model);
	};
};
