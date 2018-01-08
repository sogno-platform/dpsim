/** Python binding for ExternalCurrentSource.
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

const char *Python::Components::DocCurrentSourceIdeal =
"CurrentSourceIdeal(name, node1, node2, initial_current)\n"
"Construct a new external current source.\n"
"\n"
"An external current source is pretty much the same as a normal ideal current "
"source, but its current value can be controlled from external programs by "
"registering it with an `Interface`.\n"
"\n"
":param initial_current: The current of this source in the first timestep (as a complex value).\n"
":returns: A new `Component` representing this current source.\n";

PyObject* Python::Components::DP::CurrentSourceIdeal(PyObject* self, PyObject* args)
{
	const char *name;
	int src, dest;
	Py_complex initCurrent;

	if (!PyArg_ParseTuple(args, "siiD", &name, &src, &dest, &initCurrent))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::Components::DP::CurrentSourceIdeal>(name, src, dest, DPsim::Complex(initCurrent.real, initCurrent.imag));

	return (PyObject*) pyComp;
}
