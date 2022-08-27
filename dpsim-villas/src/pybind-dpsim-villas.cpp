// SPDX-License-Identifier: Apache-2.0

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <dpsim-villas/InterfaceVillas.h>

namespace py = pybind11;
using namespace py::literals;
using namespace villas;

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

	py::class_<PyInterfaceVillas, std::shared_ptr<PyInterfaceVillas>>(m, "InterfaceVillas", interface)
	    .def(py::init<const CPS::String&, const CPS::String&, CPS::UInt, CPS::UInt, CPS::UInt>(), "name"_a, "config"_a, "queue_length"_a=512, "sample_length"_a = 64, "downsampling"_a=1)
		.def(py::init<const CPS::String&, py::dict, CPS::UInt, CPS::UInt, CPS::UInt>(), "name"_a, "config"_a, "queue_length"_a=512, "sample_length"_a = 64, "downsampling"_a=1)
		.def("import_attribute", &PyInterfaceVillas::importAttribute, "attr"_a, "idx"_a)
		.def("export_attribute", &PyInterfaceVillas::exportAttribute, "attr"_a, "idx"_a);
}
