#include <cstdio>
#include <cstdlib>

#include <villas/sample.h>
#include <villas/shmem.h>

#include "VillasInterface.h"

using namespace DPsim;

VillasInterface::VillasInterface(const char* name) {
	this->mShmemName = name;
	mShmem = shmem_shared_open(name, &this->mBase);
	if (!mShmem) {
		std::perror("Failed to open/map shared memory object");
		std::exit(1);
	}
}

VillasInterface::~VillasInterface() {
	shmem_shared_close(mShmem, mBase);
}

void VillasInterface::readValues() {
	struct sample *sample;
	int ret = 0;
	while (ret == 0)
		ret = shmem_shared_read(mShmem, &sample, 1);
	if (ret < 0) {
		std::cerr << "Fatal error: failed to read sample from shmem interface" << std::endl;
		std::exit(1);
	}
	int sz = mExtComponents.size();
	if (sample->length < mExtComponents.size()) {
		std::cerr << "Warning: missing data in received sample" << std::endl;
		sz = sample->length;
	}
	for (int i = 0; i < sz; i++) {
		// TODO integer format?
		if (i < mExtComponents.size()) {
			ExternalVoltageSource *evs = dynamic_cast<ExternalVoltageSource*>(mExtComponents[i]);
			if (evs)
				evs->setVoltage(sample->data[i].f);

			ExternalCurrentSource *ecs = dynamic_cast<ExternalCurrentSource*>(mExtComponents[i]);
			if (ecs)
				ecs->setCurrent(sample->data[i].f);
		}
		sample_put(sample);
	}
}
