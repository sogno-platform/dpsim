#pragma once

#include <atomic>
#include <unordered_map>

#include <villas/shmem.h>

#include "ExternalInterface.h"
#include "Components/ExternalVoltageSource.h"

namespace DPsim {
	/** Implements ExternalInterface by using the shared memory interface of VILLASnode. */
	class VillasInterface : public ExternalInterface {
	private:
		const char* mShmemName;
		struct shmem_shared* mShmem;
		void* mBase;

		std::unordered_map<int, BaseComponent*> mExtComponents;
	public:
		VillasInterface(const char* name);
		~VillasInterface();
		void registerVoltageSource(ExternalVoltageSource* evs, int num);
		virtual void readValues();
	};
};
