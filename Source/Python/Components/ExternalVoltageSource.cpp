/** Python binding for ExternalVoltageSource
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

const char *Python::DocExternalVoltageSource =
"ExternalVoltageSource(name, node1, node2, initial_voltage, num)\n"
"Construct a new external voltage source.\n"
"\n"
"An external voltage source is pretty much the same as a normal ideal voltage "
"source, but its voltage value can be controlled from external programs by "
"registering it with an `Interface`.\n"
"\n"
":param initial_current: The voltage of this source in the first timestep (as a complex value).\n"
":param num: The number of this voltage source. All ideal voltage sources must "
"be identified by sequential indices, starting with 1.\n"
":returns: A new `Component` representing this voltage source.\n";
PyObject* Python::ExternalVoltageSource(PyObject* self, PyObject* args) {
	const char *name;
	int src, dest, num;
	Py_complex initVoltage;

	if (!PyArg_ParseTuple(args, "siiDi", &name, &src, &dest, &initVoltage, &num))
		return nullptr;

	Component *pyComp = PyObject_New(Component, &ComponentType);
	Component::init(pyComp);
	pyComp->comp = std::make_shared<DPsim::ExternalVoltageSource>(name, src, dest, DPsim::Complex(initVoltage.real, initVoltage.imag), num);

	return (PyObject*) pyComp;
}
