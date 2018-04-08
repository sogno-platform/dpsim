/** Python Interface
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

#ifdef _DEBUG
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
#include <Python.h>
#endif

#include "cps/Interfaces/ExternalInterface.h"

namespace DPsim {
namespace Python {

	// Thin Python wrapper around ExternalInterface
	struct Interface {
		PyObject_HEAD

		ExternalInterface *intf;

		static void dealloc(Interface*);

		static PyObject* exportCurrent(PyObject* self, PyObject* args);
		static PyObject* exportVoltage(PyObject* self, PyObject* args);
		static PyObject* registerSource(PyObject* self, PyObject* args);
	};

	extern PyTypeObject InterfaceType;

	extern const char* DocOpenShmemInterface;
	PyObject* OpenShmemInterface(PyObject *self, PyObject *args, PyObject *kwds);
}
}
