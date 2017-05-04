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
	for (ExtComponent extComp : mExtComponents) {
		if (extComp.realIdx >= sample->length || extComp.imagIdx >= sample->length) {
			std::cerr << "Fatal error: incomplete data received from shmem interface" << std::endl;
			std::exit(1);
		}
		// TODO integer format?
		Real real = sample->data[extComp.realIdx].f;
		Real imag = sample->data[extComp.imagIdx].f;

		ExternalCurrentSource *ecs = dynamic_cast<ExternalCurrentSource*>(extComp.comp);
		if (ecs)
			ecs->setCurrent(real, imag);
		ExternalVoltageSource *evs = dynamic_cast<ExternalVoltageSource*>(extComp.comp);
		if (evs)
			evs->setVoltage(real, imag);
	}
}

void VillasInterface::writeValues(SystemModel& model) {
	struct sample *sample;
	sample_alloc(&mShmem->pool, &sample, 1);
	Int len = 0;
	for (auto vd : mExportedVoltages) {
		Real real = 0, imag = 0;
		if (vd.from > 0) {
			real += model.getRealFromLeftSideVector(vd.from-1);
			imag += model.getImagFromLeftSideVector(vd.from-1);
		}
		if (vd.to > 0) {
			real -= model.getRealFromLeftSideVector(vd.to-1);
			imag -= model.getImagFromLeftSideVector(vd.to-1);
		}
		if (vd.realIdx >= sample->capacity || vd.imagIdx >= sample->capacity) {
			std::cerr << "fatal error: not enough space in allocated struct sample" << std::endl;
			std::exit(1);
		}
		sample->data[vd.realIdx].f = real;
		sample->data[vd.imagIdx].f = imag;
		if (vd.realIdx > len)
			len = vd.realIdx;
		if (vd.imagIdx > len)
			len = vd.imagIdx;
	}
	sample->length = len+1;
	sample->sequence = mSeq++;
	clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
	int ret = 0;
	while (ret == 0)
		ret = shmem_shared_write(mShmem, &sample, 1);
	if (ret < 0) {
		std::cerr << "Failed to write samples to shmem interface" << std::endl;
	}
}
