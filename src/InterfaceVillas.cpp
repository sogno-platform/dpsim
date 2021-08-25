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

#include <dpsim-villas/InterfaceVillas.h>
#include <cps/Logger.h>

using namespace CPS;
using namespace DPsim;

InterfaceVillas::InterfaceVillas(const String &name, const String &nodeType, const String &nodeConfig, UInt downsampling) :
	InterfaceSampleBased(name, name, true, downsampling), // Set sync=true for all InterfaceVillas instances
	mNodeType(nodeType),
	mNodeConfig(nodeConfig)
	{
	
	node::NodeType* nodeTypeStruct = node::node_type_lookup(mNodeType);
	if (nodeTypeStruct != nullptr) {
		mNode = std::make_unique<node::Node>(nodeTypeStruct);
		json_error_t error;
		json_t* config = json_loads(mNodeConfig.c_str(), 0, &error);
		if (config == nullptr) {
			throw JsonError(config, error);
		}

		int ret = 0;
		uuid_t fakeSuperNodeUUID;
		uuid_generate_random(fakeSuperNodeUUID);
		ret = mNode->parse(config, fakeSuperNodeUUID);
		if (ret < 0) {
			mLog->error("Error: Node in InterfaceVillas failed to parse config. Parse returned code {}", ret);
			std::exit(1);
		}
		ret = mNode->check();
		if (ret < 0) {
			mLog->error("Error: Node in InterfaceVillas failed check. Check returned code {}", ret);
			std::exit(1);
		}
		ret = mNode->prepare();
		if (ret < 0) {
			mLog->error("Error: Node in InterfaceVillas failed to prepare. Prepare returned code {}", ret);
			std::exit(1);
		}
	} else {
		mLog->error("Error: NodeType {} is not known to VILLASnode!", mNodeType);
		std::exit(1);
	}
}

void InterfaceVillas::open(CPS::Logger::Log log) {
	mLog = log;
	mLog->info("Opening InterfaceVillas...");
	int ret = mNode->start();
	if (ret < 0) {
		mLog->error("Fatal error: failed to start node in InterfaceVillas. Start returned code {}", ret);
		close();
		std::exit(1);
	}
	mOpened = true;
}

void InterfaceVillas::close() {
	mLog->info("Closing InterfaceVillas...");
	int ret = mNode->stop();
	if (ret < 0) {
		mLog->error("Error: failed to stop node in InterfaceVillas. Stop returned code {}", ret);
		std::exit(1);
	}
	mOpened = false;
}

void InterfaceVillas::readValues(bool blocking) {
	Sample *sample = nullptr;
	int ret = 0;
	try {
		while (ret == 0)
			ret = mNode->read(&sample, 1);
		if (ret < 0) {
			mLog->error("Fatal error: failed to read sample from InterfaceVillas. Read returned code {}", ret);
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

void InterfaceVillas::writeValues() {
	Sample *sample = nullptr;
	Int ret = 0;
	bool done = false;
	try {
		for (auto exp : mExports) {
			exp(sample);
		}

		sample->sequence = mSequence++;
		sample->flags |= (int) villas::node::SampleFlags::HAS_DATA;
		clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
		done = true;

		do {
			ret = mNode->write(&sample, 1);
		} while (ret == 0);
		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceShmem. Write returned code {}", ret);

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
			ret = mNode->write(&sample, 1);

		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceShmem. Write returned code {}", ret);

		/* Don't throw here, because we managed to send something */
	}
}
