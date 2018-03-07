/** Python binding for Capacitors.
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
#include "CPowerSystems/Components/DP_Line_Rx.h"

namespace DPsim {
namespace Python {
namespace Components {

	extern const char* DocLineRx;

	template<class C>
	PyObject* LineRx(PyObject* self, PyObject* args)
	{
		const char *name;
		double resistance, inductance, capacitance;
		int src, dest, type = 3;

		if (!PyArg_ParseTuple(args, "siiddd|i", &name, &src, &dest, &resistance, &inductance, &capacitance, &type))
			return nullptr;

		DPsim::Components::DP::RxLine::Type ltype;
		switch (type) {
			case 2: ltype = DPsim::Components::DP::RxLine::Node2; break;
			case 3: ltype = DPsim::Components::DP::RxLine::Node3; break;
		}

		Component *pyComp = PyObject_New(Component, &DPsim::Python::ComponentType);
		Component::init(pyComp);
		pyComp->comp = std::make_shared<C>(name, src, dest, resistance, inductance, ltype);

		return (PyObject*) pyComp;
	}
}
}
}
