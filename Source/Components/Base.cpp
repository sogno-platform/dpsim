/** Base component
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#ifndef _MSC_VER
  #include <cxxabi.h>
#endif

#include <typeinfo>

#include "Base.h"

using namespace DPsim;

String Components::Base::getType()
{
	Int status = 1;
	const char *mangled;

	mangled = typeid(*this).name();

#ifdef _MSC_VER
	String ret(mangled);

	return ret;
#else
	const char *unmangled;

	unmangled = abi::__cxa_demangle(mangled, NULL, NULL, &status);

	if (status) {
		return mangled;
	}
	else {
		String ret(unmangled);

		free((void *) unmangled);

		if (ret.find("DPsim::") == 0)
			return ret.substr(7);
		else
			return ret;
	}
#endif
}
