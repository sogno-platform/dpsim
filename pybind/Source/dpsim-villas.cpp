/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <dpsim/InterfaceShmem.h>

namespace py = pybind11;

PYBIND11_MODULE(dpsimpyvillas, m) {
	py::object interface = (py::object) py::module_::import("dpsimpy").attr("Interface");

	py::class_<DPsim::InterfaceShmem>(m, "InterfaceShmem", interface)
	    .def(py::init<const CPS::String&, const CPS::String&>(), py::arg("shmwrite") = "/dpsim-villas", py::arg("shmread") = "/villas-dpsim");

}