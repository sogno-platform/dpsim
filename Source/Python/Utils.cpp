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

// For Python PyDateTime_* functions
#include <datetime.h>

#include <dpsim/Python/Utils.h>

using namespace DPsim::Python;

int DPsim::Python::PyDateTime_DateTimeType_to_timepoint(PyObject *po, std::chrono::system_clock::time_point &tp)
{
	if (!PyDateTime_Check(po)) {
		PyErr_SetString(PyExc_TypeError, "Invalid start_at argument (must be of type datetime.datetime)");
		return -1;
	}

	PyDateTime_DateTime *pydt = (PyDateTime_DateTime *) po;

	if (pydt->hastzinfo) {
		PyErr_SetString(PyExc_TypeError, "Timezone aware datetime objects are not supported");
		return -1;
	}

	std::tm tm;

	tm.tm_sec  = PyDateTime_DATE_GET_SECOND(pydt);
	tm.tm_min  = PyDateTime_DATE_GET_MINUTE(pydt);
	tm.tm_hour = PyDateTime_DATE_GET_HOUR(pydt);
	tm.tm_mday = PyDateTime_GET_DAY(pydt);
	tm.tm_mon  = PyDateTime_GET_MONTH(pydt);
	tm.tm_year = PyDateTime_GET_YEAR(pydt);
	tm.tm_isdst = -1;

	// tm is given in system timezone
	time_t tt = std::mktime(&tm);

	std::chrono::system_clock::duration d = std::chrono::seconds(tt) + std::chrono::microseconds(PyDateTime_DATE_GET_MICROSECOND(pydt));

	tp = std::chrono::system_clock::time_point(d);

	return 0;
}

void DPsim::Python::setAttributes(CPS::AttributeList::Ptr al, PyObject *kwargs)
{
	PyObject *key, *value;
	Py_ssize_t pos = 0;

	while (PyDict_Next(kwargs, &pos, &key, &value)) {
		CPS::String name = PyUnicode_AsUTF8(key);

		auto attr = al->findAttribute(name);
		attr->fromPyObject(value);
	}
}
