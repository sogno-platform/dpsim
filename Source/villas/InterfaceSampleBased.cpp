// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <cstdio>
#include <cstdlib>

#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim/Villas/InterfaceSampleBased.h>
#include <cps/Logger.h>

using namespace CPS;
using namespace DPsim;

void InterfaceSampleBased::close() {
	mOpened = false;
}

void InterfaceSampleBased::PreStep::execute(Real time, Int timeStepCount) {
	if (!mIntf.mImportAttrs.empty()) {
		if (timeStepCount % mIntf.mDownsampling == 0)
			mIntf.readValues(mIntf.mSync);
	}	
}

void InterfaceSampleBased::PostStep::execute(Real time, Int timeStepCount) {
	if (!mIntf.mExportAttrs.empty()) {
		if (timeStepCount % mIntf.mDownsampling == 0)
			mIntf.writeValues();
	}
}

Attribute<Int>::Ptr InterfaceSampleBased::importInt(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}
	
	AttributeStatic<Int>::Ptr attr = AttributeStatic<Int>::make();
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].i);
	});
	mImportAttrs.push_back(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::INTEGER);
	return attr;
}

Attribute<Real>::Ptr InterfaceSampleBased::importReal(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Real>::Ptr attr = AttributeStatic<Real>::make();
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].f);
	});
	mImportAttrs.push_back(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::FLOAT);
	return attr;
}

Attribute<Bool>::Ptr InterfaceSampleBased::importBool(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Bool>::Ptr attr = AttributeStatic<Bool>::make();
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].b);
	});
	mImportAttrs.push_back(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::BOOLEAN);
	return attr;
}

Attribute<Complex>::Ptr InterfaceSampleBased::importComplex(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Complex>::Ptr attr = AttributeStatic<Complex>::make();
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		auto y = Complex(smp->data[idx].z.real(), smp->data[idx].z.imag());

		attr->set(y);
	});
	mImportAttrs.push_back(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::COMPLEX);
	return attr;
}

Attribute<Complex>::Ptr InterfaceSampleBased::importComplexMagPhase(UInt idx) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return nullptr;
	}

	AttributeStatic<Complex>::Ptr attr = AttributeStatic<Complex>::make();
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);
		auto  y = std::polar(z[0], z[1]);

		attr->set(y);
	});
	mImportAttrs.push_back(attr);
	mImportSignals[idx] = std::make_shared<node::Signal>("", "", node::SignalType::COMPLEX);
	return attr;
}

void InterfaceSampleBased::exportInt(Attribute<Int>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}

	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].i = **attr;
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::INTEGER);
}

void InterfaceSampleBased::exportReal(Attribute<Real>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].f = **attr;
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::FLOAT);
}

void InterfaceSampleBased::exportBool(Attribute<Bool>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}
	
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].b = **attr;
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::BOOLEAN);
}

void InterfaceSampleBased::exportComplex(Attribute<Complex>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	if (mOpened) {
		mLog->warn("InterfaceVillas has already been opened! Configuration will remain unchanged.");
		return;
	}
	
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		auto y = **attr;

		smp->data[idx].z = std::complex<float>(y.real(), y.imag());
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = std::make_shared<node::Signal>(name, unit, node::SignalType::COMPLEX);
}

CPS::Task::List InterfaceSampleBased::getTasks() {
	return CPS::Task::List({
		std::make_shared<InterfaceSampleBased::PreStep>(*this),
		std::make_shared<InterfaceSampleBased::PostStep>(*this)
	});
}