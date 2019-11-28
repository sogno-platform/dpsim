/** Python node
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

#include <dpsim/Python/Node.h>

using namespace DPsim;

CPS::TopologicalNode::List Python::nodesFromPython(PyObject* list) {
		CPS::TopologicalNode::List nodes;

		if (!PyList_Check(list))
			throw std::invalid_argument( "argument is not a list" );

		for (int i = 0; i < PyList_Size(list); i++) {
			PyObject* obj = PyList_GetItem(list, i);
			if (PyObject_TypeCheck(obj, &Python::Node<CPS::Real>::type)) {
				Node<CPS::Real>* pyNode = (Node<CPS::Real>*) obj;
				nodes.push_back(std::dynamic_pointer_cast<CPS::TopologicalNode>(pyNode->node));
			}
			else if (PyObject_TypeCheck(obj, &Python::Node<CPS::Complex>::type)) {
				Node<CPS::Complex>* pyNode = (Node<CPS::Complex>*) obj;
				nodes.push_back(std::dynamic_pointer_cast<CPS::TopologicalNode>(pyNode->node));
			}
			else {
				throw std::invalid_argument( "list element is not a dpsim.Node" );
			}
		}

		return nodes;
	}

template<typename VarType>
typename CPS::Node<VarType>::List Python::Node<VarType>::fromPython(PyObject* list)
{
	typename CPS::Node<VarType>::List nodes;

	if (!PyList_Check(list))
		throw std::invalid_argument("argument is not a list");

	for (int i = 0; i < PyList_Size(list); i++) {
		PyObject* obj = PyList_GetItem(list, i);
		if (!PyObject_TypeCheck(obj, &Python::Node<VarType>::type))
			throw std::invalid_argument("list element is not a dpsim.Node");

		Node<VarType>* pyNode = (Node<VarType>*) obj;
		nodes.push_back(pyNode->node);
	}

	return nodes;
}

template<typename VarType>
PyObject* Python::Node<VarType>::newfunc(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	Python::Node<VarType> *self;

	self = (Python::Node<VarType>*) type->tp_alloc(type, 0);
	if (self) {
		using SharedNodePtr = std::shared_ptr<CPS::Node<VarType>>;

		new (&self->node) SharedNodePtr();
	}

	return (PyObject*) self;
}

template<typename VarType>
int Python::Node<VarType>::init(Python::Node<VarType> *self, PyObject *args, PyObject *kwds)
{
	static const char *kwlist[] = {"uid", "initial_voltage", nullptr};

	const char *uid;
	CPS::Complex initialVoltage;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|D", (char **) kwlist, &uid, &initialVoltage)) {
		return -1;
	}

	self->node = std::make_shared<CPS::Node<VarType>>(uid, CPS::PhaseType::Single, std::vector<CPS::Complex>({ initialVoltage }));

	return 0;
};

template<typename VarType>
void Python::Node<VarType>::dealloc(Python::Node<VarType> *self)
{
	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the vectors here to free the associated memory.

	// This is a workaround for a compiler bug: https://stackoverflow.com/a/42647153/8178705
	using SharedNodePtr = std::shared_ptr<CPS::Node<VarType>>;

	if (self->node)
		self->node.~SharedNodePtr();

	Py_TYPE(self)->tp_free((PyObject*) self);
}

template<typename VarType>
const char* Python::Node<VarType>::docInitialVoltage =
"initial_voltage\n"
"Return the initial voltage at this node.\n";
template<typename VarType>
PyObject* Python::Node<VarType>::initialVoltage(PyObject *self, PyObject *args) {
	Python::Node<VarType>* pyNode = (Python::Node<VarType>*) self;
	CPS::Complex v = pyNode->node->initialSingleVoltage();

	return PyComplex_FromDoubles(v.real(), v.imag());
}

template<typename VarType>
const char* Python::Node<VarType>::docVoltage =
"initial_voltage\n"
"Return the initial voltage at this node.\n";
template<typename VarType>
PyObject* Python::Node<VarType>::voltage(PyObject *self, PyObject *args) {
	Python::Node<VarType>* pyNode = (Python::Node<VarType>*) self;
	CPS::Complex v = pyNode->node->singleVoltage();

	return PyComplex_FromDoubles(v.real(), v.imag());
}

template<typename VarType>
const char * Python::Node<VarType>::docGND =
"GND\n"
"Get a reference of the global ground node.\n";
template<typename VarType>
PyObject * Python::Node<VarType>::gnd(PyObject *self, PyObject *args) {
	if (!Py_GND) {
		Python::Node<VarType> *pyNode = PyObject_New(Node<VarType>, &Python::Node<VarType>::type);

		new (&pyNode->node) typename CPS::Node<VarType>::Ptr(nullptr);
		pyNode->node = CPS::Node<VarType>::GND;

		Py_GND = (PyObject *) pyNode;
	}

	Py_INCREF(Py_GND);
	return Py_GND;
}

template<typename VarType>
const char* Python::Node<VarType>::docSetInitialVoltage =
"set_initial_voltage\n"
"Set the initial voltage at this node.\n";
template<typename VarType>
PyObject* Python::Node<VarType>::setInitialVoltage(PyObject *self, PyObject *args, PyObject *kwds) {

	Py_complex initVoltage;
	CPS::Int phaseIndex;

	static const char *kwlist[] = {"voltage", "index", nullptr};

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "Di", (char **) kwlist, &initVoltage, &phaseIndex))
		Py_RETURN_NONE;

	Python::Node<VarType>* pyNode = (Python::Node<VarType>*) self;
	pyNode->node->setInitialVoltage(CPS::Complex(initVoltage.real,initVoltage.imag), phaseIndex);

	Py_RETURN_NONE;
}

template<typename VarType>
PyMethodDef Python::Node<VarType>::methods[] = {
	{"initial_voltage", (PyCFunction) Python::Node<VarType>::initialVoltage, METH_NOARGS, (char*) Python::Node<VarType>::docInitialVoltage},
	{"voltage", (PyCFunction) Python::Node<VarType>::voltage, METH_NOARGS, (char*) Python::Node<VarType>::docVoltage},
	{"GND", (PyCFunction) Python::Node<VarType>::gnd, METH_NOARGS | METH_STATIC, (char *) Python::Node<VarType>::docGND},
	{"set_initial_voltage", (PyCFunction) Python::Node<VarType>::setInitialVoltage, METH_VARARGS | METH_KEYWORDS, (char *) Python::Node<VarType>::docSetInitialVoltage},
	{nullptr, nullptr, 0, nullptr}
};

template<typename VarType>
const char* Python::Node<VarType>::doc =
"A system node.\n"
"\n"
"Proper ``__init__`` signature:\n"
"\n"
"``__init__(self, uid, initial_voltage)``.\n\n"
"``uid`` is unique identifier for this node.\n\n"
"``initial_voltage`` is the initial voltage of the node.\n\n";

template<>
const char* Python::Node<CPS::Real>::name = "dpsim.emt.Node";

template<>
const char* Python::Node<CPS::Complex>::name = "dpsim.dp.Node";

template<typename VarType>
PyTypeObject Python::Node<VarType>::type = {
	PyVarObject_HEAD_INIT(nullptr, 0)
	(char *) Python::Node<VarType>::name,    /* tp_name */
	sizeof(Python::Node<VarType>),           /* tp_basicsize */
	0,                                       /* tp_itemsize */
	(destructor)Python::Node<VarType>::dealloc, /* tp_dealloc */
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
	(char *) Python::Node<VarType>::doc,     /* tp_doc */
	0,                                       /* tp_traverse */
	0,                                       /* tp_clear */
	0,                                       /* tp_richcompare */
	0,                                       /* tp_weaklistoffset */
	0,                                       /* tp_iter */
	0,                                       /* tp_iternext */
	Python::Node<VarType>::methods,          /* tp_methods */
	0,                                       /* tp_members */
	0,                                       /* tp_getset */
	0,                                       /* tp_base */
	0,                                       /* tp_dict */
	0,                                       /* tp_descr_get */
	0,                                       /* tp_descr_set */
	0,                                       /* tp_dictoffset */
	(initproc) Python::Node<VarType>::init,  /* tp_init */
	0,                                       /* tp_alloc */
	Python::Node<VarType>::newfunc,     /* tp_new */
};

template struct Python::Node<CPS::Real>;
template struct Python::Node<CPS::Complex>;

