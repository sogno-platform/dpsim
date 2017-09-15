#include "PyComponent.h"

#include "CIMReader.h"

using namespace DPsim;

PyTypeObject DPsim::PyComponentType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Component",                 /* tp_name */
	sizeof(PyComponent),               /* tp_basicsize */
	0,                                 /* tp_itemsize */
	(destructor)PyComponent::dealloc,  /* tp_dealloc */
	0,                                 /* tp_print */
	0,                                 /* tp_getattr */
	0,                                 /* tp_setattr */
	0,                                 /* tp_reserved */
	0,                                 /* tp_repr */
	0,                                 /* tp_as_number */
	0,                                 /* tp_as_sequence */
	0,                                 /* tp_as_mapping */
	0,                                 /* tp_hash  */
	0,                                 /* tp_call */
	0,                                 /* tp_str */
	0,                                 /* tp_getattro */
	0,                                 /* tp_setattro */
	0,                                 /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT |
		Py_TPFLAGS_BASETYPE,           /* tp_flags */
	"A component in a simulation.",    /* tp_doc */
	0,                                 /* tp_traverse */
	0,                                 /* tp_clear */
	0,                                 /* tp_richcompare */
	0,                                 /* tp_weaklistoffset */
	0,                                 /* tp_iter */
	0,                                 /* tp_iternext */
	0,                                 /* tp_methods */
	0,                                 /* tp_members */
	0,                                 /* tp_getset */
	0,                                 /* tp_base */
	0,                                 /* tp_dict */
	0,                                 /* tp_descr_get */
	0,                                 /* tp_descr_set */
	0,                                 /* tp_dictoffset */
	0,                                 /* tp_init */
	0,                                 /* tp_alloc */
	PyComponent::newfunc,              /* tp_new */
};

PyObject* PyComponent::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds) {
	PyComponent* self = (PyComponent*) type->tp_alloc(type, 0);
	if (self)
		self->comp = nullptr;
	return (PyObject*) self;
}

void PyComponent::dealloc(PyComponent* self) {
	if (self->comp)
		delete self->comp;
	Py_TYPE(self)->tp_free((PyObject*)self);
}

bool DPsim::compsFromPython(PyObject* list, std::vector<BaseComponent*>& comps) {
	if (!PyList_Check(list))
		return false;
	for (int i = 0; i < PyList_Size(list); i++) {
		PyObject* obj = PyList_GetItem(list, i);
		if (!PyObject_TypeCheck(obj, &PyComponentType)) {
			comps.clear();
			return false;
		}
		PyComponent* pyComp = (PyComponent*) obj;
		comps.push_back(pyComp->comp);
	}
	return true;
}

PyObject* DPsim::pyLoadCim(PyObject* self, PyObject* args) {
	double frequency = 50;
	PyObject *list;
	PyBytesObject *filename;
	CIMReader *reader;

	if (PyArg_ParseTuple(args, "O&|d", PyUnicode_FSConverter, &filename, &frequency)) {
		reader = new CIMReader(2*PI*frequency);
		reader->addFile(PyBytes_AsString((PyObject*) filename));
		Py_DECREF(filename);
	} else if (PyArg_ParseTuple(args, "O|d", &list, &frequency)) {
		PyErr_Clear();
		if (!PyList_Check(list)) {
			PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
			return nullptr;
		}
		reader = new CIMReader(2*PI*frequency);
		for (int i = 0; i < PyList_Size(list); i++) {
			if (!PyUnicode_FSConverter(PyList_GetItem(list, i), &filename)) {
				delete reader;
				PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
				return nullptr;
			}	
			reader->addFile(PyBytes_AsString((PyObject*) filename));
			Py_DECREF(filename);
		}
	} else {
		PyErr_SetString(PyExc_TypeError, "First argument must be filename or list of filenames");
		return nullptr;
	}
	reader->parseFiles();
	std::vector<BaseComponent*> comps = reader->getComponents();
	list = PyList_New(comps.size());
	for (int i = 0; i < comps.size(); i++) {
		// corresponds to "pyComp = dpsim.Component()" in Python
		PyObject *emptyTuple = PyTuple_New(0);
		PyComponent* pyComp = (PyComponent*) PyObject_CallObject((PyObject*) &PyComponentType, emptyTuple);
		Py_DECREF(emptyTuple);
		if (!pyComp) {
			Py_DECREF(list);
			delete reader;
			return nullptr;
		}
		/*
		PyComponent* pyComp = PyObject_New(PyComponent, &PyComponentType);
		PyObject_Init((PyObject*) pyComp, &PyComponentType);
		*/
		pyComp->comp = comps[i];
		PyList_SET_ITEM(list, i, (PyObject*) pyComp);
	}
	delete reader;
	return list;
}
