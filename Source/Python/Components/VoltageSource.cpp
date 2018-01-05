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

#include "Component.h"

using namespace DPsim;

const char *Python::Components::DocVoltageSource =
"VoltSourceRes(name, node1, node2, voltage, resistance)\n"
"Construct a new voltage source with an internal resistance.\n"
"\n"
"Because this is actually internally represented as an equivalent current "
"source, it does **not** count towards the numbering of ideal voltage sources.\n"
"\n"
"Attributes: ``resistance``, ``voltage``.\n"
"\n"
":param voltage: Complex voltage in Volt.\n"
":param resistance: Internal resistance in Ohm.\n"
":returns: A new `Component` representing this voltage source.\n";
PyObject* Python::Components::DP::VoltageSource(PyObject* self, PyObject* args)
{
	const char *name;
	double resistance;
	int src, dest;
	Py_complex voltage;

	if (!PyArg_ParseTuple(args, "siiDd", &name, &src, &dest, &voltage, &resistance))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::Component::DP::VoltageSource>(name, src, dest, Complex(voltage.real, voltage.imag), resistance);

	return (PyObject*) pyComp;
}
