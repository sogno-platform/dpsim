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
	mSeq = 0;
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

void VillasInterface::writeValues(SystemModel& model) {
	struct sample *sample;
	sample_alloc(&mShmem->pool, &sample, 1);
	int len = mExportedVoltages.size();
	if (sample->capacity < len) {
		std::cerr << "struct sample returned from pool has to small capacity" << std::endl;
		len = sample->capacity;
	}
	sample->length = len;
	Matrix lvect = model.getLeftSideVector();
	for (int i = 0; i < len; i++) {
		Real f = 0.0f;
		VoltDiff vd = mExportedVoltages[i];
		if (vd.from > 0)
			f += lvect(vd.from-1, 0);
		if (vd.to > 0)
			f -= lvect(vd.to-1, 0);
		sample->data[i].f = f;
	}
	sample->sequence = mSeq++;
	clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
	int ret = 0;
	while (ret == 0)
		ret = shmem_shared_write(mShmem, &sample, 1);
	if (ret < 0) {
		std::cerr << "Failed to write samples to shmem interface" << std::endl;
	}
}
