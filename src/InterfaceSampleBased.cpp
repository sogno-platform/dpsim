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

#include <dpsim-villas/InterfaceSampleBased.h>
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
	Attribute<Int>::Ptr attr = Attribute<Int>::make(Flags::read | Flags::write);
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].i);
	});
	mImportAttrs.push_back(attr);
	return attr;
}

Attribute<Real>::Ptr InterfaceSampleBased::importReal(UInt idx) {
	Attribute<Real>::Ptr attr = Attribute<Real>::make(Flags::read | Flags::write);
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].f);
	});
	mImportAttrs.push_back(attr);
	return attr;
}

Attribute<Bool>::Ptr InterfaceSampleBased::importBool(UInt idx) {
	Attribute<Bool>::Ptr attr = Attribute<Bool>::make(Flags::read | Flags::write);
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		attr->set(smp->data[idx].b);
	});
	mImportAttrs.push_back(attr);
	return attr;
}

Attribute<Complex>::Ptr InterfaceSampleBased::importComplex(UInt idx) {
	Attribute<Complex>::Ptr attr = Attribute<Complex>::make(Flags::read | Flags::write);
	auto& log = mLog;
	addImport([attr, idx, log](Sample *smp) {
		if (idx >= smp->length) {
			log->error("incomplete data received from InterfaceVillas");
			return;
		}
		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);
		auto  y = Complex(z[0], z[1]);

		attr->set(y);
	});
	mImportAttrs.push_back(attr);
	return attr;
}

Attribute<Complex>::Ptr InterfaceSampleBased::importComplexMagPhase(UInt idx) {
	Attribute<Complex>::Ptr attr = Attribute<Complex>::make(Flags::read | Flags::write);
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
	return attr;
}

void InterfaceSampleBased::exportInt(Attribute<Int>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].i = attr->getByValue();
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = Signal(idx, villas::node::SignalType::INTEGER, name, unit);
}

void InterfaceSampleBased::exportReal(Attribute<Real>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].f = attr->getByValue();
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = Signal(idx, villas::node::SignalType::FLOAT, name, unit);
}

void InterfaceSampleBased::exportBool(Attribute<Bool>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		smp->data[idx].b = attr->getByValue();
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = Signal(idx, villas::node::SignalType::BOOLEAN, name, unit);
}

void InterfaceSampleBased::exportComplex(Attribute<Complex>::Ptr attr, UInt idx, const std::string &name, const std::string &unit) {
	addExport([attr, idx](Sample *smp) {
		if (idx >= smp->capacity)
			throw std::out_of_range("not enough space in allocated sample");
		if (idx >= smp->length)
			smp->length = idx + 1;

		auto  y = attr->getByValue();
		auto *z = reinterpret_cast<float*>(&smp->data[idx].z);

		z[0] = y.real();
		z[1] = y.imag();
	});
	mExportAttrs.push_back(attr);
	mExportSignals[idx] = Signal(idx, villas::node::SignalType::COMPLEX, name, unit);
}

CPS::Task::List InterfaceSampleBased::getTasks() {
	return CPS::Task::List({
		std::make_shared<InterfaceSampleBased::PreStep>(*this),
		std::make_shared<InterfaceSampleBased::PostStep>(*this)
	});
}
