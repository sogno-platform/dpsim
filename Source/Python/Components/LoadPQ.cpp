/** Python binding for Resistors.
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

#include "Python/Components/LoadPQ.h"

const char *DPsim::Python::Components::DocLoadPQ =
"LoadPQ(name, node, activePower, reactivePower, volt, angle)\n"
"Construct a new PQ load.\n"
"\n"
":param activePower: Active Power in VA.\n"
":param reactivePower: Reactive Power in VAr.\n"
":returns: A new `Component` representing this PQ load.\n";

template<>
PyObject* DPsim::Python::Components::LoadPQ<CPS::DP::Ph1::PQLoad>(PyObject* self, PyObject* args)
{
    const char *name;
    double activePower, reactivePower, volt;

    if (!PyArg_ParseTuple(args, "sddd", &name, &activePower, &reactivePower, &volt))
        return nullptr;

    try {
        Component *pyComp = PyObject_New(Component, &DPsim::Python::ComponentType);
        Component::init(pyComp);
        pyComp->comp = std::make_shared<CPS::DP::Ph1::PQLoad>(name/*, activePower, reactivePower, volt*/);

        return (PyObject*) pyComp;
    } catch (...) {
        return nullptr;
    }
}
