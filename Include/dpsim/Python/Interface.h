/** Python Interface
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/Definitions.h>

namespace DPsim {
#ifdef WITH_SHMEM
	class Interface;
#endif

namespace Python {

	// Thin Python wrapper around Interface
	struct Interface {
		PyObject_HEAD

#ifdef WITH_SHMEM
		CPS::Interface::Config conf;
		CPS::Interface::Ptr intf;
		const char *wname, *rname;
		PyObject *pyExports;
#endif
		static void addExportDesc(Interface *self, int idx, const CPS::String &type, const CPS::String &name, const CPS::String &suffix = "");

		static void dealloc(Interface *self);

		static int init(Interface *self, PyObject *args, PyObject *kwds);
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static PyObject* addImport(Interface *self, PyObject *args, PyObject *kwargs);
		static PyObject* addExport(Interface *self, PyObject *args, PyObject *kwargs);
		static PyObject* exports(Interface *self, void *ctx);

		static PyMethodDef methods[];
		static PyMemberDef members[];
		static PyTypeObject type;
		static const char* doc;
		static const char* docOpen;
		static const char* docAddImport;
		static const char* docAddExport;
	};
}
}
