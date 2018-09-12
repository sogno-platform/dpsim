/** Python utilities
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

#include <ctime>

#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <dpsim/Python/Utils.h>

using namespace DPsim::Python;

void DPsim::Python::setAttributes(CPS::AttributeList::Ptr al, PyObject *kwargs)
{
	PyObject *key, *value;
	Py_ssize_t pos = 0;

	while (PyDict_Next(kwargs, &pos, &key, &value)) {
		CPS::String name = PyUnicode_AsUTF8(key);

		auto attr = al->attribute(name);
		attr->fromPyObject(value);
	}
}
