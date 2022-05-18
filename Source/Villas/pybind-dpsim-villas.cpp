/* Copyright 2017-2022 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <dpsim/Villas/InterfaceShmem.h>
#include <dpsim/Villas/InterfaceVillas.h>

namespace py = pybind11;
using namespace py::literals;
using namespace villas;

class PyInterfaceShmem : public DPsim::InterfaceShmem {

public:
	using DPsim::InterfaceShmem::InterfaceShmem;

	py::dict getConfig() {
		auto signals = std::list<py::dict>();

		int maxIdx = 0;
		for (auto const& a : mExportSignals) {
			if (a.first > maxIdx)
				maxIdx = a.first;
		}

		for (int i = 0; i <= maxIdx; i++) {
			node::Signal::Ptr s;
			try {
				s = mExportSignals.at(i);
			} catch(std::out_of_range &) {
				s = std::make_shared<node::Signal>("", "", node::SignalType::FLOAT);
			}

			auto signal = py::dict(
				"name"_a = s->name,
				"type"_a = node::signalTypeToString(s->type)
			);

			if (!s->unit.empty()) {
				signal["unit"] = s->unit;
			}

			signals.push_back(signal);
		}

		return py::dict(
			"type"_a = "shmem",
			"queuelen"_a = mConf.queuelen,
			"samplelen"_a = mConf.samplelen,
			"mode"_a = mConf.polling ? "polling" : "pthread",
			"in"_a = py::dict(
				"name"_a = mWName,
				"signals"_a = signals
			),
			"out"_a = py::dict(
				"name"_a = mRName
			)
		);
	}
};

class PyInterfaceVillas: public DPsim::InterfaceVillas {

public:
	using DPsim::InterfaceVillas::InterfaceVillas;

	PyInterfaceVillas(const CPS::String &name, py::dict config, CPS::UInt queueLength, CPS::UInt sampleLength, CPS::UInt downsampling) :
		InterfaceVillas(
			name, 
			(py::str) py::module_::import("json").attr("dumps")(config, "indent"_a = py::none()), //json.dumps(config, indent=None)
			queueLength,
			sampleLength,
			downsampling)
		{}
};



PYBIND11_MODULE(dpsimpyvillas, m) {
	py::object interface = (py::object) py::module_::import("dpsimpy").attr("Interface");

	py::class_<PyInterfaceShmem>(m, "InterfaceShmem", interface)
	    .def(py::init<const CPS::String&, const CPS::String&>(), py::arg("shmwrite") = "/dpsim-villas", py::arg("shmread") = "/villas-dpsim")
		.def("get_config", &PyInterfaceShmem::getConfig);

	py::class_<PyInterfaceVillas>(m, "InterfaceVillas", interface)
	    .def(py::init<const CPS::String&, const CPS::String&, CPS::UInt, CPS::UInt, CPS::UInt>(), "name"_a, "config"_a, "queue_length"_a=512, "sample_length"_a = 64, "downsampling"_a=1)
		.def(py::init<const CPS::String&, py::dict, CPS::UInt, CPS::UInt, CPS::UInt>(), "name"_a, "config"_a, "queue_length"_a=512, "sample_length"_a = 64, "downsampling"_a=1);
}
