/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <stdexcept>
#include <cstdio>
#include <cstdlib>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim/Villas/InterfaceVillas.h>
#include <cps/Logger.h>
#include <villas/signal_list.hpp>
#include <villas/path.hpp>

using namespace CPS;
using namespace DPsim;

Bool InterfaceVillas::villasInitialized = false;
UInt InterfaceVillas::villasAffinity = 0;
UInt InterfaceVillas::villasPriority = 0;
UInt InterfaceVillas::villasHugePages = 100;

InterfaceVillas::InterfaceVillas(const String &name, const String &nodeConfig, UInt queueLength, UInt sampleLength, UInt downsampling) :
	InterfaceSampleBased(name, name, true, downsampling), // Set sync=true for all InterfaceVillas instances
	mNodeConfig(nodeConfig),
	mQueueLength(queueLength),
	mSampleLength(sampleLength)
	{ }

void InterfaceVillas::open(CPS::Logger::Log log) {
	mLog = log;
	mLog->info("Opening InterfaceVillas...");

	if (!InterfaceVillas::villasInitialized) {
		mLog->info("Initializing Villas...");
		initVillas();
		InterfaceVillas::villasInitialized = true;
	}

	json_error_t error;
	json_t* config = json_loads(mNodeConfig.c_str(), 0, &error);
	if (config == nullptr) {
		throw JsonError(config, error);
	}

	json_t* nodeType = json_object_get(config, "type");
	if (nodeType == nullptr) {
		mLog->error("Error: Node config does not contain type-key!");
		std::exit(1);
	}
	String nodeTypeString = json_string_value(nodeType);

	mNode = node::NodeFactory::make(nodeTypeString);

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

	mLog->info("Preparing VILLASNode instance...");
	setupNodeSignals();
	prepareNode();
	mLog->info("Node is ready to send / receive data!");
	mOpened = true;

	mSequence = 0;
	mLastSample = node::sample_alloc(&mSamplePool);
	mLastSample->signals = mNode->getInputSignals(false);
	mLastSample->sequence = 0;
	mLastSample->ts.origin.tv_sec = 0;
	mLastSample->ts.origin.tv_nsec = 0;

	std::memset(&mLastSample->data, 0, mLastSample->capacity * sizeof(float));
}


void InterfaceVillas::prepareNode() {
	int ret = node::pool_init(&mSamplePool, mQueueLength, sizeof(Sample) + SAMPLE_DATA_LENGTH(mSampleLength));
	if (ret < 0) {
		mLog->error("Error: InterfaceVillas failed to init sample pool. pool_init returned code {}", ret);
		std::exit(1);
	}

	ret = mNode->prepare();
	if (ret < 0) {
		mLog->error("Error: Node in InterfaceVillas failed to prepare. Prepare returned code {}", ret);
		std::exit(1);
	}

	mNode->getFactory()->start(nullptr); //We have no SuperNode, so just hope type_start doesnt use it...

	ret = mNode->start();
	if (ret < 0) {
		mLog->error("Fatal error: failed to start node in InterfaceVillas. Start returned code {}", ret);
		close();
		std::exit(1);
	}
}

void InterfaceVillas::setupNodeSignals() {
	mNode->out.path = new node::Path();
	mNode->out.path->signals = std::make_shared<node::SignalList>();
	node::SignalList::Ptr nodeOutputSignals = mNode->out.path->getOutputSignals(false);
	nodeOutputSignals->clear();
	int idx = 0;
	for (auto sig : mExportSignals) {
		while (sig.first > idx) {
			nodeOutputSignals->push_back(std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
			idx++;
		}
		nodeOutputSignals->push_back(sig.second);
		idx++;
	}

	node::SignalList::Ptr nodeInputSignals = mNode->getInputSignals(false);
	nodeInputSignals->clear();
	idx = 0;
	for (auto sig : mImportSignals) {
		while (sig.first > idx) {
			nodeInputSignals->push_back(std::make_shared<node::Signal>("", "", node::SignalType::INVALID));
			idx++;
		}
		nodeInputSignals->push_back(sig.second);
		idx++;
	}
}

void InterfaceVillas::close() {
	mLog->info("Closing InterfaceVillas...");
	int ret = mNode->stop();
	if (ret < 0) {
		mLog->error("Error: failed to stop node in InterfaceVillas. Stop returned code {}", ret);
		std::exit(1);
	}
	mOpened = false;
	ret = node::pool_destroy(&mSamplePool);
	if (ret < 0) {
		mLog->error("Error: failed to destroy SamplePool in InterfaceVillas. pool_destroy returned code {}", ret);
		std::exit(1);
	}

	mNode->getFactory()->stop();

	delete mNode;
}

void InterfaceVillas::readValues(bool blocking) {
	Sample *sample = node::sample_alloc(&mSamplePool);
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
		sample = node::sample_alloc(&mSamplePool);
		if (sample == nullptr) {
			mLog->error("InterfaceVillas could not allocate a new sample! Not sending any data!");
			return;
		}

		sample->signals = mNode->getOutputSignals(false);

		for (auto exp : mExports) {
			exp(sample);
		}

		sample->sequence = mSequence++;
		sample->flags |= (int) villas::node::SampleFlags::HAS_SEQUENCE;
		sample->flags |= (int) villas::node::SampleFlags::HAS_DATA;
		clock_gettime(CLOCK_REALTIME, &sample->ts.origin);
		sample->flags |= (int) villas::node::SampleFlags::HAS_TS_ORIGIN;
		done = true;

		do {
			ret = mNode->write(&sample, 1);
		} while (ret == 0);
		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceVillas. Write returned code {}", ret);

		sample_copy(mLastSample, sample);
		sample_decref(sample);
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

		sample_decref(sample);

		if (ret < 0)
			mLog->error("Failed to write samples to InterfaceVillas. Write returned code {}", ret);

		/* Don't throw here, because we managed to send something */
	}
}

void InterfaceVillas::initVillas() {
	int ret = node::memory::init(villasHugePages);
	if (ret)
		throw RuntimeError("Error: VillasNode failed to initialize memory system");

	villas::kernel::rt::init(villasPriority, villasAffinity);
}