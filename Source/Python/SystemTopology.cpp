/** Python system tolopogy
 *
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

#include <dpsim/Python/SystemTopology.h>
#include <dpsim/Python/Component.h>
#include <dpsim/Python/Node.h>

#include "structmember.h"

using namespace DPsim;

const char *Python::SystemTopology::docAddComponent =
"add_component(comp)\n"
"Add a component to this system topology\n";
PyObject* Python::SystemTopology::addComponent(SystemTopology *self, PyObject *args)
{
	PyObject* pyObj;
	PyObject *pyName;
	Python::Component* pyComp;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;

	if (!PyObject_TypeCheck(pyObj, &Python::Component::type)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Component");
		return nullptr;
	}

	pyComp = (Python::Component *) pyObj;

	pyName = PyUnicode_FromString(pyComp->comp->name().c_str());
	if (PyDict_Contains(self->pyComponentDict, pyName)) {
		PyErr_SetString(PyExc_TypeError, "SystemTopology already contains a component with this name");
		return nullptr;
	}

	pyComp = (Component *) pyObj;
	self->sys->addComponent(pyComp->comp);

	PyDict_SetItem(self->pyComponentDict, pyName, pyObj);

	Py_DECREF(pyName);
	Py_DECREF(pyObj);

	Py_RETURN_NONE;
}

const char *Python::SystemTopology::docAddNode =
"add_node(comp)\n"
"Add a node to this system topology\n";
PyObject* Python::SystemTopology::addNode(SystemTopology *self, PyObject *args)
{
	PyObject* pyObj;
	PyObject *pyName;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;

	CPS::TopologicalNode::Ptr topoNode;
	if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Real>::type)) {
		auto pyNode = (Python::Node<CPS::Real> *) pyObj;

		topoNode = std::dynamic_pointer_cast<CPS::TopologicalNode>(pyNode->node);
	}
	else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Complex>::type)) {
		auto pyNode = (Python::Node<CPS::Complex> *) pyObj;

		topoNode = std::dynamic_pointer_cast<CPS::TopologicalNode>(pyNode->node);
	}
	else {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Node");
		return nullptr;
	}

	pyName = PyUnicode_FromString(topoNode->name().c_str());
	if (PyDict_Contains(self->pyNodeDict, pyName)) {
		PyErr_SetString(PyExc_TypeError, "SystemTopology already contains a node with this name");
		return nullptr;
	}

	self->sys->addNode(topoNode);

	PyDict_SetItem(self->pyNodeDict, pyName, pyObj);

	Py_DECREF(pyName);
	Py_DECREF(pyObj);

	Py_RETURN_NONE;
}

const char *Python::SystemTopology::docMultiply =
"multiply(n)\n"
"Add n copies of the topology (all components, nodes and their connections) "
"to itself.\n";
PyObject* Python::SystemTopology::multiply(SystemTopology *self, PyObject *args) {
	int n;

	if (!PyArg_ParseTuple(args, "i", &n))
		return nullptr;

	self->sys->multiply(n);

	Py_RETURN_NONE;
}

#ifdef WITH_GRAPHVIZ
const char *Python::SystemTopology::docReprSVG =
"_repr_svg_(comp)\n"
"Return a SVG graph rendering of the system Topology\n";
PyObject* Python::SystemTopology::reprSVG(SystemTopology *self, PyObject *args)
{
	Python::SystemTopology *pySys = (Python::SystemTopology*) self;

	auto graph = pySys->sys->topologyGraph();

	std::stringstream ss;

	graph.render(ss, "neato", "svg");

	return PyUnicode_FromString(ss.str().c_str());
}
#endif

PyObject* Python::SystemTopology::newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	Python::SystemTopology *self;

	self = (Python::SystemTopology*) type->tp_alloc(type, 0);
	if (self) {
		using SharedSysPtr = std::shared_ptr<CPS::SystemTopology>;

		new (&self->sys) SharedSysPtr();

		self->pyComponentDict = PyDict_New();
		self->pyNodeDict = PyDict_New();
	}

	return (PyObject*) self;
}

int Python::SystemTopology::init(Python::SystemTopology *self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"frequency", "nodes", "components", nullptr};
	double frequency = 50;

	PyObject *pyCompList = nullptr;
	PyObject *pyNodeList = nullptr;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "d|OO", (char **) kwlist, &frequency, &pyNodeList, &pyCompList)) {
		return -1;
	}

	CPS::Component::List compList;
	CPS::TopologicalNode::List nodeList;

	if (pyNodeList) {
		try {
			nodeList = nodesFromPython(pyNodeList);
		} catch (...) {
			PyErr_Format(PyExc_AttributeError, "Parameter 'nodes' must be a list of dpsim.Node");
			return -1;
		}

		for (int i = 0; i < PyList_Size(pyNodeList); i++) {
			PyObject *pyObj = PyList_GetItem(pyNodeList, i);
			PyObject *pyName;

			if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Complex>::type)) {
				Python::Node<CPS::Complex> *pyNode = (Python::Node<CPS::Complex> *) pyObj;
				pyName = PyUnicode_FromString(pyNode->node->name().c_str());
			}
			else if (PyObject_TypeCheck(pyObj, &Python::Node<CPS::Real>::type)) {
				Python::Node<CPS::Real> *pyNode = (Python::Node<CPS::Real> *) pyObj;
				pyName = PyUnicode_FromString(pyNode->node->name().c_str());
			}
			else {
				PyErr_Format(PyExc_AttributeError, "Parameter 'nodes' must be a list of dpsim.Node");
				return -1;
			}

			if (PyDict_Contains(self->pyNodeDict, pyName)) {
				PyErr_Format(PyExc_AttributeError, "Duplicated Node name");
				return -1;
			}

			PyDict_SetItem(self->pyNodeDict, pyName, pyObj);
		}
	}

	if (pyCompList) {
		try {
			compList = compsFromPython(pyCompList);
		} catch (...) {
			PyErr_Format(PyExc_AttributeError, "Parameter 'components' must be a list of dpsim.Component");
			return -1;
		}

		for (int i = 0; i < PyList_Size(pyCompList); i++) {
			PyObject *pyObj = PyList_GetItem(pyCompList, i);
			Python::Component *pyComp = (Python::Component *) pyObj;
			PyObject *pyName = PyUnicode_FromString(pyComp->comp->name().c_str());

			if (PyDict_Contains(self->pyComponentDict, pyName)) {
				PyErr_Format(PyExc_AttributeError, "Duplicated Component name");
				return -1;
			}

			PyDict_SetItem(self->pyComponentDict, pyName, pyObj);
		}
	}

	self->sys = std::make_shared<CPS::SystemTopology>(frequency, nodeList, compList);

	return 0;
}

void Python::SystemTopology::dealloc(Python::SystemTopology *self)
{
	Py_DECREF(self->pyComponentDict);
	Py_DECREF(self->pyNodeDict);

	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the vectors here to free the associated memory.

	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using SharedSysPtr = std::shared_ptr<CPS::SystemTopology>;

	if (self->sys)
		self->sys.~SharedSysPtr();

	Py_TYPE(self)->tp_free((PyObject*) self);
}

const char *Python::SystemTopology::docComponents = "";
const char *Python::SystemTopology::docNodes = "";

PyMemberDef Python::SystemTopology::members[] = {
	{(char*) "components", T_OBJECT, offsetof(Python::SystemTopology, pyComponentDict), READONLY, (char*) Python::SystemTopology::docComponents},
	{(char*) "nodes",      T_OBJECT, offsetof(Python::SystemTopology, pyNodeDict),      READONLY, (char*) Python::SystemTopology::docNodes},
	{nullptr, 0, 0, 0, nullptr}
};

PyMethodDef Python::SystemTopology::methods[] = {
	{"add_component", (PyCFunction) Python::SystemTopology::addComponent, METH_VARARGS, Python::SystemTopology::docAddComponent},
	{"add_node",      (PyCFunction) Python::SystemTopology::addNode, METH_VARARGS, Python::SystemTopology::docAddNode},
	{"multiply",      (PyCFunction) Python::SystemTopology::multiply, METH_VARARGS, Python::SystemTopology::docMultiply},
#ifdef WITH_GRAPHVIZ
	{"_repr_svg_",    (PyCFunction) Python::SystemTopology::reprSVG, METH_NOARGS, Python::SystemTopology::docReprSVG},
#endif
	{nullptr, nullptr, 0, nullptr}
};

const char* Python::SystemTopology::doc =
"A system topology.\n"
"\n"
"Proper ``__init__`` signature:\n"
"\n"
"``__init__(self, frequency=50.0, components)``.\n\n"
"``frequency`` is the nominal system frequency in Hz.\n\n"
"``components`` must be a list of `Component` that are to be simulated.\n\n";
PyTypeObject Python::SystemTopology::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	"dpsim.SystemTopology",                  /* tp_name */
	sizeof(Python::SystemTopology),          /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::SystemTopology::dealloc, /* tp_dealloc */
	0,                                       /* tp_print */
	0,                                       /* tp_getattr */
	0,                                       /* tp_setattr */
	0,                                       /* tp_reserved */
	0,                                       /* tp_repr */
	0,                                       /* tp_as_number */
	0,                                       /* tp_as_sequence */
	0,                                       /* tp_as_mapping */
	0,                                       /* tp_hash  */
	0,                                       /* tp_call */
	0,                                       /* tp_str */
	0,                                       /* tp_getattro */
	0,                                       /* tp_setattro */
	0,                                       /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,/* tp_flags */
	Python::SystemTopology::doc,             /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Python::SystemTopology::methods,         /* tp_methods */
	Python::SystemTopology::members,         /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc)Python::SystemTopology::init,  /* tp_init */
	0,                                       /* tp_alloc */
	Python::SystemTopology::newfunc,         /* tp_new */
};
