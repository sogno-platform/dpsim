/** Python components
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <vector>
#include <memory>

#include <cps/PowerComponent.h>
#include <cps/Attribute.h>

#include <dpsim/Python/Node.h>
#include <dpsim/Python/Utils.h>

namespace DPsim {
namespace Python {

	extern PyTypeObject ComponentType;

	struct Component {
		PyObject_HEAD

		CPS::Component::Ptr comp;

		// List of additional objects that aren't directly used from Simulation
		// methods, but that a reference has be kept to to avoid them from being
		// freed (e.g. Interfaces).
		std::vector<PyObject*> refs;

		static void init(Component* self);

		static PyObject* newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds);
		static void dealloc(Component*);

		static PyObject* str(Component* self);

		static PyObject* getattr(Component* self, char* name);
		static int setattr(Component *self, char* name, PyObject *v);

		static PyObject* connect(Component* self, PyObject *args);

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
				Component *pyComp = PyObject_New(Component, &ComponentType);
				Component::init(pyComp);

				// Create CPS component
				auto comp = std::make_shared<T>(name, name);

				// Set parameters
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
			catch (const CPS::TypeException &e) {
				PyErr_SetString(PyExc_ValueError, "Invalid attribute type");
				return nullptr;
			}
			catch (const CPS::InvalidAttributeException &e) {
				PyErr_SetString(PyExc_ValueError, "Invalid attribute");
				return nullptr;
			}
			catch (const CPS::Exception &e) {
				PyErr_SetString(PyExc_ValueError, e.what());
				return nullptr;
			}
		}

		template<typename T>
		static const char * getDocumentation()
		{
			std::stringstream doc;

			T comp("uid", "name");

			doc << comp.type() << "(name, nodes, **attributes)" << std::endl
			    << "Construct a new component with a given name an list of nodes." << std::endl;
#if 0
			    << comp.getDescription() << std::endl
			    << std::endl;

			for (auto& it : comp.getAttributes()) {
				auto name = it.first;
				auto attr = it.second;

				if (!(attr->getFlags() & CPS::Flags::write))
					continue;

				doc << ":param " << name << ": " << attr->getDescription() << std::endl;
			}

			    << ":returns: A new `Component` representing this " << comp.getType() << "." << std::endl;
#endif

			auto docstr = new CPS::String(doc.str());

			return docstr->c_str();
		}

		template<typename T>
		static PyMethodDef getConstructorDef(const char *name)
		{
			return {
				name,
				(PyCFunction) createInstance<T>,
				METH_VARARGS | METH_KEYWORDS,
				getDocumentation<T>()
			};
		}
	};

	CPS::Component::List compsFromPython(PyObject* list);
}
}
