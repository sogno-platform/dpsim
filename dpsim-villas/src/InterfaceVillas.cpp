// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <cstdio>
#include <cstdlib>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim-villas/InterfaceVillas.h>
#include <dpsim-models/Logger.h>
#include <villas/signal_list.hpp>
#include <villas/path.hpp>

using namespace CPS;
using namespace DPsim;

Bool InterfaceVillas::villasInitialized = false;
UInt InterfaceVillas::villasAffinity = 0;
UInt InterfaceVillas::villasPriority = 0;
UInt InterfaceVillas::villasHugePages = 100;

InterfaceVillas::InterfaceVillas(const String &nodeConfig, UInt queueLength, UInt sampleLength) :
	Interface(),
	mNodeConfig(nodeConfig),
	mQueueLength(queueLength),
	mSampleLength(sampleLength)
	{ }

void InterfaceVillas::open() {
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

void InterfaceVillas::readValuesFromEnv(CPS::AttributeBase::List& updatedAttrs) {
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
			updatedAttrs.push_back(imp(sample));
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

void InterfaceVillas::writeValuesToEnv(CPS::AttributeBase::List& updatedAttrs) {
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
			//TODO: At this point, we need to have a full list of queued attributes corresponding to an export lambda each
			exp(updatedAttrs[0], sample); //FIXME: This will crash!
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


Attribute<Int>::Ptr InterfaceVillas::importInt(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}
	
	AttributeStatic<Int>::Ptr attr = AttributeStatic<Int>::make();
	auto& log = mLog;
	addImport([idx, log](Sample *smp) -> AttributeBase::Ptr {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		return AttributePointer<AttributeBase>(AttributeStatic<Int>::make(smp->data[idx].i));
	});
	tryGetManager()->importAttribute(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::INTEGER);
	return attr;
}

Attribute<Real>::Ptr InterfaceVillas::importReal(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Real>::Ptr attr = AttributeStatic<Real>::make();
	auto& log = mLog;
	addImport([idx, log](Sample *smp) -> AttributeBase::Ptr {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		return AttributePointer<AttributeBase>(AttributeStatic<Real>::make(smp->data[idx].f));
	});
	tryGetManager()->importAttribute(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::FLOAT);
	return attr;
}

Attribute<Bool>::Ptr InterfaceVillas::importBool(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Bool>::Ptr attr = AttributeStatic<Bool>::make();
	auto& log = mLog;
	addImport([idx, log](Sample *smp) -> AttributeBase::Ptr {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		return AttributePointer<AttributeBase>(AttributeStatic<Bool>::make(smp->data[idx].b));
	});
	tryGetManager()->importAttribute(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::BOOLEAN);
	return attr;
}

Attribute<Complex>::Ptr InterfaceVillas::importComplex(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Complex>::Ptr attr = AttributeStatic<Complex>::make();
	auto& log = mLog;
	addImport([idx, log](Sample *smp) -> AttributeBase::Ptr {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		auto y = Complex(smp->data[idx].z.real(), smp->data[idx].z.imag());

		return AttributePointer<AttributeBase>(AttributeStatic<Complex>::make(y));
	});
	tryGetManager()->importAttribute(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::COMPLEX);
	return attr;
}

Attribute<Complex>::Ptr InterfaceVillas::importComplexMagPhase(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Complex>::Ptr attr = AttributeStatic<Complex>::make();
	auto& log = mLog;
	addImport([idx, log](Sample *smp) -> AttributeBase::Ptr {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);
		auto  y = std::polar(z[0], z[1]);

		return AttributePointer<AttributeBase>(AttributeStatic<Complex>::make(y));
	});
	tryGetManager()->importAttribute(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::COMPLEX);
	return attr;
}

void InterfaceVillas::exportInt(Attribute<Int>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}

	addExport([idx](AttributeBase::Ptr attr, Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		Attribute<Int>::Ptr attrTyped = std::dynamic_pointer_cast<Attribute<Int>>(attr.getPtr());

		if (attrTyped.isNull()) {
			throw InvalidAttributeException();
		}

		smp->data[idx].i = **attrTyped;
	});
	tryGetManager()->exportAttribute(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::INTEGER);
}

void InterfaceVillas::exportReal(Attribute<Real>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([idx](AttributeBase::Ptr attr, Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		Attribute<Real>::Ptr attrTyped = std::dynamic_pointer_cast<Attribute<Real>>(attr.getPtr());

		if (attrTyped.isNull()) {
			throw InvalidAttributeException();
		}

		smp->data[idx].f = **attrTyped;
	});
	tryGetManager()->exportAttribute(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::FLOAT);
}

void InterfaceVillas::exportBool(Attribute<Bool>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}
	
	addExport([idx](AttributeBase::Ptr attr, Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		Attribute<Bool>::Ptr attrTyped = std::dynamic_pointer_cast<Attribute<Bool>>(attr.getPtr());

		if (attrTyped.isNull()) {
			throw InvalidAttributeException();
		}

		smp->data[idx].b = **attrTyped;
	});
	tryGetManager()->exportAttribute(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::BOOLEAN);
}

void InterfaceVillas::exportComplex(Attribute<Complex>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}
	
	addExport([idx](AttributeBase::Ptr attr, Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		Attribute<Complex>::Ptr attrTyped = std::dynamic_pointer_cast<Attribute<Complex>>(attr.getPtr());

		if (attrTyped.isNull()) {
			throw InvalidAttributeException();
		}

		auto y = **attrTyped;

		smp->data[idx].z = std::complex<float>(y.real(), y.imag());
	});
	tryGetManager()->exportAttribute(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::COMPLEX);
}
