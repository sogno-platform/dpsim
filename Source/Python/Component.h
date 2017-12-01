/** Python components
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
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
#include "Components/BaseComponent.h"

namespace DPsim {
namespace Python {

	struct Component {
		PyObject_HEAD

		ElementPtr comp;

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(Component*);

		static PyObject* str(Component* self);

		static PyObject* getattr(Component* self, char* name);
		static int setattr(Component *self, char* name, PyObject *v);
	};

	extern PyTypeObject ComponentType;

	bool compsFromPython(PyObject* list, ElementList& comps);

	// "Constructors" for the component types
	extern const char* DocExternalCurrentSource;
	PyObject* ExternalCurrentSource(PyObject* self, PyObject *args);
	extern const char* DocExternalVoltageSource;
	PyObject* ExternalVoltageSource(PyObject* self, PyObject *args);
	extern const char* DocInductor;
	PyObject* Inductor(PyObject* self, PyObject *args);
	extern const char* DocResistor;
	PyObject* Resistor(PyObject* self, PyObject *args);
	extern const char* DocVoltSourceRes;
	PyObject* VoltSourceRes(PyObject* self, PyObject *args);

	extern const char* DocLoadCim;
	PyObject* LoadCim(PyObject* self, PyObject* args);
};
};
