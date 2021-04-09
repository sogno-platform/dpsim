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

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <dpsim-villas/InterfaceShmem.h>

namespace py = pybind11;

PYBIND11_MODULE(dpsimpyvillas, m) {
	py::object interface = (py::object) py::module_::import("dpsimpy").attr("Interface");

	py::class_<DPsim::InterfaceShmem>(m, "InterfaceShmem", interface)
	    .def(py::init<const CPS::String&, const CPS::String&>(), py::arg("shmwrite") = "/dpsim-villas", py::arg("shmread") = "/villas-dpsim");

}