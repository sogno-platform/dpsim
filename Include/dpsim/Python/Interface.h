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

namespace DPsim {
#ifdef WITH_SHMEM
	class Interface;
#endif

namespace Python {

	// Thin Python wrapper around Interface
	struct Interface {
		PyObject_HEAD

#ifdef WITH_SHMEM
		CPS::Interface::Ptr intf;
#endif

		static void dealloc(Interface*);

		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static PyObject* registerControlledAttribute(Interface* self, PyObject* args);
		static PyObject* registerExportedAttribute(Interface* self, PyObject* args);
	};

	extern PyTypeObject InterfaceType;

	extern const char* DocOpenInterface;
	PyObject* OpenInterface(PyObject *self, PyObject *args, PyObject *kwds);
}
}
