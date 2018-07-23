/** Python node
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#pragma once

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <cps/Node.h>

namespace CPS {
namespace Python {

	template<typename VarType>
	struct Node {
		PyObject_HEAD

		std::shared_ptr<CPS::Node<VarType>> node;

		// The Python API has no notion of C++ classes and methods, so the methods
		// that can be called from Python are static.
		//
		// Helper methods for memory management / initialization etc.
		static PyObject* newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds);
		static int init(Node<VarType> *self, PyObject *args, PyObject *kwds);
		static void dealloc(Node<VarType> *self);

		static PyObject * getGND(PyObject *self, PyObject *args);

		static char *name;
		static PyMethodDef methods[];
		static PyTypeObject type;
		static char* DocGND;
		static char* Doc;
		
		static PyObject *Py_GND;

		static typename CPS::Node<VarType>::List fromPython(PyObject* list);
	};

	template<typename VarType>
	PyObject *Node<VarType>::Py_GND = NULL;

	typename CPS::NodeBase::List nodesFromPython(PyObject* list) {
		CPS::NodeBase::List nodes;

		if (!PyList_Check(list))
			throw std::invalid_argument( "argument is not a list" );

		for (int i = 0; i < PyList_Size(list); i++) {
			PyObject* obj = PyList_GetItem(list, i);
			if (PyObject_TypeCheck(obj, &Python::Node<Real>::type)) {
				Node<Real>* pyNode = (Node<Real>*) obj;
				nodes.push_back(std::dynamic_pointer_cast<NodeBase>(pyNode->node));
			}
			else if (PyObject_TypeCheck(obj, &Python::Node<Complex>::type)) {
				Node<Complex>* pyNode = (Node<Complex>*) obj;
				nodes.push_back(std::dynamic_pointer_cast<NodeBase>(pyNode->node));
			}
			else {
				throw std::invalid_argument( "list element is not a dpsim.Node" );		
			}			
		}

		return nodes;
	}
}
}
