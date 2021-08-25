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

#include <stdexcept>
#include <cstdio>
#include <cstdlib>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim-villas/InterfaceShmem.h>
#include <cps/Logger.h>

using namespace CPS;
using namespace DPsim;

void InterfaceShmem::open(CPS::Logger::Log log) {
	mLog = log;

	mLog->info("Opening InterfaceShmem: {} <-> {}", mWName, mRName);

	if (shmem_int_open(mWName.c_str(), mRName.c_str(), &mShmem, &mConf) < 0) {
		mLog->error("Failed to open/map shared memory object");
		std::exit(1);
	}

	mLog->info("Opened InterfaceShmem: {} <-> {}", mWName, mRName);

	mSequence = 0;

	if (shmem_int_alloc(&mShmem, &mLastSample, 1) < 0) {
		mLog->info("Failed to allocate single sample from pool");
		close();
		std::exit(1);
	}

	mLastSample->sequence = 0;
	mLastSample->ts.origin.tv_sec = 0;
	mLastSample->ts.origin.tv_nsec = 0;

	std::memset(&mLastSample->data, 0, mLastSample->capacity * sizeof(float));
}

void InterfaceShmem::close() {
	shmem_int_close(&mShmem);
	InterfaceSampleBased::close();
}

void InterfaceShmem::readValues(bool blocking) {
	Sample *sample = nullptr;
	int ret = 0;
	try {
		if (!blocking) {
			// Check if theres actually data available
			ret = queue_signalled_available(&mShmem.read.shared->queue);
			if (ret <= 0)
				return;

			ret = shmem_int_read(&mShmem, &sample, 1);
			if (ret == 0)
				return;
		}
		else {
			while (ret == 0)
				ret = shmem_int_read(&mShmem, &sample, 1);
		}
		if (ret < 0) {
			mLog->error("Fatal error: failed to read sample from InterfaceShmem");
			close();
			std::exit(1);
		}

		for (auto imp : mImports) {
			imp(sample);
		}

		sample_decref(sample);
	}
	catch (std::exception& exc) {
		/* probably won't happen (if the timer expires while we're still reading data,
		 * we have a bigger problem somewhere else), but nevertheless, make sure that
		 * we're not leaking memory from the queue pool */
		if (sample)
			sample_decref(sample);

		throw exc;
	}
}

void InterfaceShmem::writeValues() {
	Sample *sample = nullptr;
	Int ret = 0;
	bool done = false;
	try {
		if (shmem_int_alloc(&mShmem, &sample, 1) < 1) {
			mLog->error("Fatal error: pool underrun in: {} <-> {} at sequence no {}", mWName, mRName, mSequence);
			close();
			std::exit(1);
		}

		for (auto exp : mExports) {
			exp(sample);
		}

		sample->sequence = mSequence++;
		sample->flags |= (int) villas::node::SampleFlags::HAS_DATA;
		clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
		done = true;

		do {
			ret = shmem_int_write(&mShmem, &sample, 1);
		} while (ret == 0);
		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceShmem");

		sample_copy(mLastSample, sample);
	}
	catch (std::exception& exc) {
		/* We need to at least send something, so determine where exactly the
		 * timer expired and either resend the last successfully sent sample or
		 * just try to send this one again.
		 * TODO: can this be handled better? */
		if (!done)
			sample = mLastSample;

		while (ret == 0)
			ret = shmem_int_write(&mShmem, &sample, 1);

		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceShmem");

		/* Don't throw here, because we managed to send something */
	}
}
