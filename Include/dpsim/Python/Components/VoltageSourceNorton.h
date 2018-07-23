/** Python binding for VoltSourceRes.
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#pragma once

#include "Python/Component.h"
#include "Python/Node.h"
#include "cps/Components/DP_VoltageSource_Norton.h"
#include "cps/Components/EMT_VoltageSource_Norton.h"

namespace CPS {
namespace Python {
namespace Components {

	extern const char* DocVoltageSourceNorton;

	template<class C>
	PyObject* VoltageSourceNorton(PyObject* self, PyObject* args);

	template<>
	PyObject* VoltageSourceNorton<CPS::Components::EMT::VoltageSourceNorton>(PyObject* self, PyObject* args) {
		const char *name;
		double resistance;

		Py_complex voltage;
		PyObject *pyNodes;

		if (!PyArg_ParseTuple(args, "sODd", &name, &pyNodes, &voltage, &resistance))
			return nullptr;

		try {
			CPS::Node<Real>::List nodes = Python::Node<Real>::fromPython(pyNodes);

			Component *pyComp = PyObject_New(Component, &ComponentType);
			Component::init(pyComp);

			String *mystring = new String(name);

			pyComp->comp = std::make_shared<CPS::Components::EMT::VoltageSourceNorton>(*mystring, nodes, Complex(voltage.real, voltage.imag), resistance);

			return (PyObject*) pyComp;
		} catch (...) {
			return nullptr;
		}
	}

	template<>
	PyObject* VoltageSourceNorton<CPS::Components::DP::VoltageSourceNorton>(PyObject* self, PyObject* args) {
		const char *name;
		double resistance;

		Py_complex voltage;
		PyObject *pyNodes;

		if (!PyArg_ParseTuple(args, "sODd", &name, &pyNodes, &voltage, &resistance))
			return nullptr;

		try {
			CPS::Node<Complex>::List nodes = Python::Node<Complex>::fromPython(pyNodes);

			Component *pyComp = PyObject_New(Component, &ComponentType);
			Component::init(pyComp);

			String *mystring = new String(name);

			pyComp->comp = std::make_shared<CPS::Components::DP::VoltageSourceNorton>(*mystring, nodes, Complex(voltage.real, voltage.imag), resistance);

			return (PyObject*) pyComp;
		} catch (...) {
			return nullptr;
		}
	}
}
}
}
