/** Python components
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <Python.h>

#include <vector>
#include <memory>

#include "Simulation.h"
#include "Components/Base.h"

namespace DPsim {
namespace Python {

	struct Component {
		PyObject_HEAD

		DPsim::Components::Base::Ptr comp;
		static void init(Component* self);

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(Component*);

		static PyObject* str(Component* self);

		static PyObject* getattr(Component* self, char* name);
		static int setattr(Component *self, char* name, PyObject *v);
	};

	extern PyTypeObject ComponentType;

	bool compsFromPython(PyObject* list, DPsim::Components::Base::List& comps);

namespace Components {
	extern const char* DocCurrentSource;
	extern const char* DocVoltageSource;
	extern const char* DocResistor;
	extern const char* DocInductor;
	extern const char* DocCapacitor;
	extern const char* DocVoltageSourceNorton;

namespace DP {
	PyObject* CurrentSource(PyObject* self, PyObject *args);
	PyObject* VoltageSource(PyObject* self, PyObject *args);
	PyObject* VoltageSourceNorton(PyObject* self, PyObject *args);
	PyObject* Resistor(PyObject* self, PyObject *args);
	PyObject* Inductor(PyObject* self, PyObject *args);
	PyObject* Capacitor(PyObject* self, PyObject *args);
}

namespace EMT {
	PyObject* Resistor(PyObject* self, PyObject *args);
	PyObject* Inductor(PyObject* self, PyObject *args);
} /// EMT
} /// Components
} /// Python
} /// DPsim
