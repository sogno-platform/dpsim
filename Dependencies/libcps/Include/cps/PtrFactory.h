/** Shared Factory
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <memory>
#include <utility>

/// Curiously recurring template pattern (CRTP) to create create new shared_ptr instances.
/// See: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
template<typename T>
class SharedFactory {

public:
	template<typename... Args>
	static std::shared_ptr<T> make(Args&&... args) {
		return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
	}
};

template<typename T>
class UniqueFactory {

public:

	template<typename... Args>
	static std::unique_ptr<T> make(Args&&... args) {
		return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
	}
};
