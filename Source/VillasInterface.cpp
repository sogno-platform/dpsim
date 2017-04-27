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

void VillasInterface::registerVoltageSource(ExternalVoltageSource *evs, int num) {
	mExtComponents[num] = evs;
}

void VillasInterface::registerCurrentSource(ExternalCurrentSource *ecs, int num) {
	mExtComponents[num] = ecs;
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
	for (auto it = mExtComponents.begin(); it != mExtComponents.end(); ++it) {
		if (sample->length <= it->first) {
			std::cerr << "Warning: missing data in received sample" << std::endl;
			continue;
		}
		// TODO integer format?
		ExternalVoltageSource *evs = dynamic_cast<ExternalVoltageSource*>(it->second);
		if (evs)
			evs->setVoltage(sample->data[it->first].f);

		ExternalCurrentSource *ecs = dynamic_cast<ExternalCurrentSource*>(it->second);
		if (ecs)
			ecs->setCurrent(sample->data[it->first].f);
		sample_put(sample);
	}
}
