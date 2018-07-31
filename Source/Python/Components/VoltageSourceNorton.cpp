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

#include "Python/Components/VoltageSourceNorton.h"

const char *DPsim::Python::Components::DocVoltageSourceNorton =
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

template<>
PyObject* DPsim::Python::Components::VoltageSourceNorton<CPS::EMT::Ph1::VoltageSourceNorton>(PyObject* self, PyObject* args) {
    const char *name;
    double resistance;

    Py_complex voltage;
    PyObject *pyNodes;

    if (!PyArg_ParseTuple(args, "sODd", &name, &pyNodes, &voltage, &resistance))
        return nullptr;

    try {
        CPS::Node<CPS::Real>::List nodes = Python::Node<CPS::Real>::fromPython(pyNodes);

        Component *pyComp = PyObject_New(Component, &ComponentType);
        Component::init(pyComp);

        CPS::String *mystring = new CPS::String(name);

        pyComp->comp = std::make_shared<CPS::EMT::Ph1::VoltageSourceNorton>(*mystring, nodes, CPS::Complex(voltage.real, voltage.imag), resistance);

        return (PyObject*) pyComp;
    } catch (...) {
        return nullptr;
    }
}

template<>
PyObject* DPsim::Python::Components::VoltageSourceNorton<CPS::DP::Ph1::VoltageSourceNorton>(PyObject* self, PyObject* args) {
    const char *name;
    double resistance;

    Py_complex voltage;
    PyObject *pyNodes;

    if (!PyArg_ParseTuple(args, "sODd", &name, &pyNodes, &voltage, &resistance))
        return nullptr;

    try {
        CPS::Node<CPS::Complex>::List nodes = Python::Node<CPS::Complex>::fromPython(pyNodes);

        Component *pyComp = PyObject_New(Component, &ComponentType);
        Component::init(pyComp);

        CPS::String *mystring = new CPS::String(name);

        pyComp->comp = std::make_shared<CPS::DP::Ph1::VoltageSourceNorton>(*mystring, nodes, CPS::Complex(voltage.real, voltage.imag), resistance);

        return (PyObject*) pyComp;
    } catch (...) {
        return nullptr;
    }
}