/** Python system tolopogy
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <dpsim/Config.h>
#include <cps/SystemTopology.h>

namespace DPsim {
namespace Python {

	struct SystemTopology {
		PyObject_HEAD

		CPS::SystemTopology::Ptr sys;

		PyObject *pyNodeDict;
		PyObject *pyComponentDict;

		// Helper functions to update the python dicts after a C++ method
		// adds components or nodes.
		void addCppComponent(CPS::Component::Ptr comp);
		void addCppNode(CPS::TopologicalNode::Ptr node);
		void updateDicts();

		static PyObject* addComponent(SystemTopology *self, PyObject *args);
		static PyObject* addNode(SystemTopology *self, PyObject *args);
		static PyObject* addDecouplingLine(SystemTopology *self, PyObject *args);
		static PyObject* multiply(SystemTopology *self, PyObject *args);
#ifdef WITH_GRAPHVIZ
		static PyObject* reprSVG(SystemTopology *self, PyObject *args);
		static const char *docReprSVG;
#endif

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(SystemTopology *self, PyObject *args, PyObject *kwds);
		static void dealloc(SystemTopology *self);

		static const char *doc;
		static const char *docNodes;
		static const char *docComponents;
		static const char *docAddComponent;
		static const char *docAddNode;
		static const char *docAddDecouplingLine;
		static const char *docMultiply;
		static PyTypeObject type;
		static PyMethodDef methods[];
		static PyMemberDef members[];
	};
}
}
