/** Python binding for Resistors.
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

#include "Python/Component.h"

using namespace DPsim;

const char *Python::Components::DocResistor =
"Resistor(name, node1, node2, resistance)\n"
"Construct a new resistor.\n"
"\n"
"Attributes: ``resistance``.\n"
"\n"
":param resistance: Resistance in Ohm.\n"
":returns: A new `Component` representing this resistor.\n";

template<class C>
PyObject* Python::Components::Resistor(PyObject* self, PyObject* args)
{
	const char *name;
	double resistance;
	int src, dest;

	if (!PyArg_ParseTuple(args, "siid", &name, &src, &dest, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<C>(name, src, dest, resistance);

	return (PyObject*) pyComp;
}
