/** External interface
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <iostream>
#include <stdexcept>

#include <dpsim/Interface.h>
#include <cps/Logger.h>

#include <cstdio>
#include <cstdlib>

using namespace CPS;
using namespace DPsim;

void Interface::open() {
	std::cout << Logger::prefix() << "Opening interface: " <<  mWName << " <-> " << mRName << std::endl;

	if (shmem_int_open(mWName.c_str(), mRName.c_str(), &mShmem, &mConf) < 0) {
		std::perror("Failed to open/map shared memory object");
		std::exit(1);
	}

	std::cout << Logger::prefix() << "Opened interface: " <<  mWName << " <-> " << mRName << std::endl;

	mSequence = 0;

	if (shmem_int_alloc(&mShmem, &mLastSample, 1) < 0) {
		std::cout << Logger::prefix() << "Failed to allocate single sample from pool" << std::endl;
		close();
		std::exit(1);
	}

	mLastSample->sequence = 0;
	mLastSample->ts.origin.tv_sec = 0;
	mLastSample->ts.origin.tv_nsec = 0;

	std::memset(&mLastSample->data, 0, mLastSample->capacity * sizeof(float));
}

void Interface::close() {
	shmem_int_close(&mShmem);
}

void Interface::readValues(bool blocking) {
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
			std::cerr << Logger::prefix() << "Fatal error: failed to read sample from interface" << std::endl;
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

void Interface::writeValues() {
	Sample *sample = nullptr;
	Int ret = 0;
	bool done = false;
	try {
		if (shmem_int_alloc(&mShmem, &sample, 1) < 1) {
			std::cerr << Logger::prefix() << "Fatal error: pool underrun in: " << mWName << " <-> " << mRName;
			std::cerr << " at sequence no " << mSequence << std::endl;
			close();
			std::exit(1);
		}

		for (auto exp : mExports) {
			exp(sample);
		}

		sample->sequence = mSequence++;
		sample->flags |= SAMPLE_HAS_DATA;
		clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
		done = true;

		do {
			ret = shmem_int_write(&mShmem, &sample, 1);
		} while (ret == 0);
		if (ret < 0)
			std::cerr << Logger::prefix() << "Failed to write samples to interface" << std::endl;

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
			std::cerr << Logger::prefix() << "Failed to write samples to interface" << std::endl;

		/* Don't throw here, because we managed to send something */
	}
}

void Interface::addImport(Attribute<Int>::Ptr attr, Int idx) {
	addImport([attr, idx](Sample *smp) {
		if (idx >= smp->length)
			throw std::length_error("incomplete data received from interface");

		attr->set(smp->data[idx].i);
	});
}

void Interface::addImport(Attribute<Real>::Ptr attr, Int idx) {
	addImport([attr, idx](Sample *smp) {
		if (idx >= smp->length)
			throw std::length_error("incomplete data received from interface");

		attr->set(smp->data[idx].f);
	});
}

void Interface::addImport(Attribute<Bool>::Ptr attr, Int idx) {
	addImport([attr, idx](Sample *smp) {
		if (idx >= smp->length)
			throw std::length_error("incomplete data received from interface");

		attr->set(smp->data[idx].b);
	});
}

void Interface::addImport(Attribute<Complex>::Ptr attr, Int idx) {
	addImport([attr, idx](Sample *smp) {
		if (idx >= smp->length)
			throw std::length_error("incomplete data received from interface");

		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);
		auto  y = Complex(z[0], z[1]);

		attr->set(y);
	});
}

void Interface::addExport(Attribute<Int>::Ptr attr, Int idx) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].i = attr->get();
	});
}

void Interface::addExport(Attribute<Real>::Ptr attr, Int idx) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].f = attr->get();
	});
}

void Interface::addExport(Attribute<Bool>::Ptr attr, Int idx) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].b = attr->get();;
	});
}

void Interface::addExport(Attribute<Complex>::Ptr attr, Int idx) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		auto  y = attr->get();;
		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);

		z[0] = y.real();
		z[1] = y.imag();
	});
}
