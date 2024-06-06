// SPDX-License-Identifier: Apache-2.0

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <dpsim-villas/InterfaceVillas.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, CPS::AttributePointer<T>);

namespace py = pybind11;
using namespace py::literals;
using namespace villas;

class PyInterfaceVillas : public DPsim::InterfaceVillas {

public:
  using DPsim::InterfaceVillas::InterfaceVillas;

  PyInterfaceVillas(py::dict config, CPS::UInt queueLength,
                    CPS::UInt sampleLength, const CPS::String &name,
                    CPS::UInt downsampling)
      : InterfaceVillas(
            (py::str)py::module_::import("json").attr("dumps")(
                config,
                "indent"_a = py::none()), //json.dumps(config, indent=None)
            queueLength, sampleLength, name, downsampling) {}
};

PYBIND11_MODULE(dpsimpyvillas, m) {
  py::object interface =
      (py::object)py::module_::import("dpsimpy").attr("Interface");

  py::class_<PyInterfaceVillas, std::shared_ptr<PyInterfaceVillas>>(
      m, "InterfaceVillas", interface)
      .def(py::init<const CPS::String &, CPS::UInt, CPS::UInt,
                    const CPS::String &, CPS::UInt>(),
           "config"_a, "queue_length"_a = 512, "sample_length"_a = 64,
           "name"_a = "", "downsampling"_a = 1)
      .def(py::init<py::dict, CPS::UInt, CPS::UInt, const CPS::String &,
                    CPS::UInt>(),
           "config"_a, "queue_length"_a = 512, "sample_length"_a = 64,
           "name"_a = "", "downsampling"_a = 1)
      .def("import_attribute", &PyInterfaceVillas::importAttribute, "attr"_a,
           // cppcheck-suppress assignBoolToPointer
           "idx"_a, "block_on_read"_a = false, "sync_on_start"_a = true,
           "name"_a = "", "unit"_a = "")
      .def("export_attribute", &PyInterfaceVillas::exportAttribute, "attr"_a,
           // cppcheck-suppress assignBoolToPointer
           "idx"_a, "wait_for_on_write"_a = true, "name"_a = "", "unit"_a = "");
}
