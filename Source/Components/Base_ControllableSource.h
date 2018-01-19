/** Current source base class
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#pragma once

#include <memory>

#include "SystemModel.h"

namespace DPsim {
namespace Components {

	class ControllableSourceBase {

	public:

		typedef std::shared_ptr<ControllableSourceBase> Ptr;

		virtual void setSourceValue(Real value) {
			setSourceValue(value);
		}

		virtual void setSourceValue(Complex value) = 0;
	};
}
}
