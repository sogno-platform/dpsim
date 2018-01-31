/** Python binding for VoltSourceRes.
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *
 * DPsim
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

namespace DPsim {
namespace Python {
namespace Components {

	extern const char* DocVoltageSourceNorton;

	template<class C>
	PyObject* VoltageSourceNorton(PyObject* self, PyObject* args)
	{
		const char *name;
		double resistance;
		int src, dest;
		Py_complex voltage;

		if (!PyArg_ParseTuple(args, "siiDd", &name, &src, &dest, &voltage, &resistance))
			return nullptr;

		Component *pyComp = PyObject_New(Component, &ComponentType);
		Component::init(pyComp);
		pyComp->comp = std::make_shared<C>(name, src, dest, Complex(voltage.real, voltage.imag), resistance);

		return (PyObject*) pyComp;
	}
}
}
}
