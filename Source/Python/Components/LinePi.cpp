/** Python binding for Capacitors.
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

#include "Python/Components/LinePi.h"

const char *DPsim::Python::Components::DocLinePi =
"LinePi(name, node1, node2, resistance, inductance, capacitance)\n"
"Construct a new Pi line.\n"
"\n"
"Attributes: ``resistance``, ``inductance``, ``capacitance``.\n"
"\n"
":param resistance: Resistance in Ohm.\n"
":param inductance: Inductance in Henry.\n"
":param capacitance: Capacitance in Farad.\n"
":returns: A new `Component` representing this Pi line.\n";


template<>
PyObject* DPsim::Python::Components::LinePi<CPS::DP::Ph1::PiLine>(PyObject* self, PyObject* args)
{
    const char *name;
    double resistance, inductance, capacitance, conductance;

    PyObject *pyNodes;

    if (!PyArg_ParseTuple(args, "sOdddd", &name, &pyNodes, &resistance, &inductance, &capacitance, &conductance))
        return nullptr;

    try {
        CPS::Node<CPS::Complex>::List nodes = Python::Node<CPS::Complex>::fromPython(pyNodes);

        Component *pyComp = PyObject_New(Component, &DPsim::Python::ComponentType);
        Component::init(pyComp);
        pyComp->comp = std::make_shared<CPS::DP::Ph1::PiLine>(name);

        return (PyObject*) pyComp;
    } catch (...) {
        return nullptr;
    }
}