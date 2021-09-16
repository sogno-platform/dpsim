/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <vector>
#include <memory>

#include <cps/SimPowerComp.h>
#include <cps/Attribute.h>

#include <dpsim/Utils.h>
#include <dpsim/Python/Node.h>
#include <dpsim/Python/Utils.h>

namespace DPsim {
namespace Python {

	struct Component {
		PyObject_HEAD

		CPS::IdentifiedObject::Ptr comp;

		static void init(Component* self);

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(Component*);

		static PyObject* str(Component* self);

		static PyObject* getattro(Component* self, PyObject *name);
		static int setattro(Component *self, PyObject *name, PyObject *v);

		static PyObject* connect(Component* self, PyObject *args);

		static PyObject* dir(Component* self, PyObject* args);

		template<typename T>
		static PyObject* createInstance(PyObject* self, PyObject* args, PyObject *kwargs)
		{
			int ret;
			const char *name;

			PyObject *pyNodes = nullptr;

			ret = PyArg_ParseTuple(args, "s|O", &name, &pyNodes);
			if (!ret)
				return nullptr;

			try {
				// Create Python wrapper
				Component *pyComp = PyObject_New(Component, &Component::type);
				Component::init(pyComp);

				// Create CPS component
				auto comp = std::make_shared<T>(name, name);

				// Set parameters
				if (kwargs)
					setAttributes(comp, kwargs);

				// Set nodes
				if (pyNodes) {
					auto nodes = Python::Node<typename T::Type>::fromPython(pyNodes);

					comp->connect(nodes);
				}

				pyComp->comp = comp;

				return (PyObject*) pyComp;
			}
			catch (const CPS::AccessException &) {
				PyErr_SetString(PyExc_ValueError, "Attribute access is prohibited");
				return nullptr;
			}
			catch (const CPS::TypeException &) {
				PyErr_SetString(PyExc_ValueError, "Invalid attribute type");
				return nullptr;
			}
			catch (const CPS::InvalidAttributeException &) {
				PyErr_SetString(PyExc_ValueError, "Invalid attribute");
				return nullptr;
			}
			catch (const CPS::Exception &e) {
				PyErr_SetString(PyExc_ValueError, e.what());
				return nullptr;
			}
		}

		template<typename T>
		static const char * documentation()
		{
			std::stringstream doc;

			doc << Utils::type<T>() << "(name, nodes, **attributes)" << std::endl
			    << "Construct a new component with a given name and list of nodes." << std::endl;

			auto docstr = new CPS::String(doc.str());

			return docstr->c_str();
		}

		template<typename T>
		static PyMethodDef constructorDef(const char *name)
		{
			return {
				name,
				(PyCFunction) createInstance<T>,
				METH_VARARGS | METH_KEYWORDS,
				documentation<T>()
			};
		}

		static const char* doc;
		static const char* docConnect;
		static PyMethodDef methods[];
		static PyTypeObject type;
	};

	CPS::IdentifiedObject::List compsFromPython(PyObject* list);
}
}
