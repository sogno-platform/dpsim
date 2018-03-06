/** Attributes
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

#include "Definitions.h"
#include "PtrFactory.h"
#include "Config.h"

#ifdef WITH_PYTHON
#include <Python.h>
#endif

namespace DPsim {
namespace Flags {
	enum Access : int {
		read = 1,
		write = 2
	};
}

	class AttributeBase {

#ifdef WITH_PYTHON
	public:
		virtual void fromPyObject(PyObject *po) = 0;
		virtual PyObject * toPyObject() = 0;
#endif

	protected:
		int mFlags;

		AttributeBase(int flags) :
			mFlags(flags) { };

	public:
		typedef std::shared_ptr<AttributeBase> Ptr; /** @todo Use unqique_ptr here? */
		typedef std::map<String, Ptr> Map;

		class AccessException { };
		class TypeException { };
	};

	template<class T>
	class Attribute : public AttributeBase, public SharedFactory<Attribute<T>> {
		friend class SharedFactory<Attribute<T>>;

#ifdef WITH_PYTHON
	public:
		virtual void fromPyObject(PyObject *po);
		virtual PyObject * toPyObject();
#endif

	protected:

		T *mValue;

	public:
		Attribute(T *v, int flags = Flags::read) :
			AttributeBase(flags),
			mValue(v)
		{ };

		void set(const T &v) {
			// Check access
			if (mFlags & Flags::write)
				*mValue = v;
			else
				throw AccessException();
		}

		T get() const {
			// Check access
			if (mFlags & Flags::read)
				return *mValue;
			else
				throw AccessException();
		}
	};
}
