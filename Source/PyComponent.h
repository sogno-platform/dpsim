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

#include "Components/BaseComponent.h"

namespace DPsim {

	struct PyComponent {
		PyObject_HEAD

		BaseComponent* comp;

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(PyComponent*);

		static PyObject* str(PyComponent* self);

		static PyObject* getattr(PyComponent* self, char* name);
		static int setattr(PyComponent *self, char* name, PyObject *v);
	};

	extern PyTypeObject PyComponentType;

	bool compsFromPython(PyObject* list, std::vector<BaseComponent*>& comps);

	// "Constructors" for the component types
	extern const char* pyDocExternalCurrentSource;
	PyObject* pyExternalCurrentSource(PyObject* self, PyObject *args);
	extern const char* pyDocExternalVoltageSource;
	PyObject* pyExternalVoltageSource(PyObject* self, PyObject *args);
	extern const char* pyDocInductor;
	PyObject* pyInductor(PyObject* self, PyObject *args);
	extern const char* pyDocLinearResistor;
	PyObject* pyLinearResistor(PyObject* self, PyObject *args);
	extern const char* pyDocVoltSourceRes;
	PyObject* pyVoltSourceRes(PyObject* self, PyObject *args);

	extern const char* pyDocLoadCim;
	PyObject* pyLoadCim(PyObject* self, PyObject* args);
};
