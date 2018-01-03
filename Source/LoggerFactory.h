/** Logger Factory
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

#include <map>
#include <memory>
#include <list>
#include <iterator>
#include <iostream>
#include "Logger.h"

namespace DPsim {
	class LoggerFactory {

		typedef std::map<std::string, std::unique_ptr<Logger>>  Container;
		typedef Container::iterator                             iterator;
		typedef Container::const_iterator                       const_iterator;
		typedef Container::value_type                           value_type;		
		//typedef std::list<std::reference_wrapper<LoggingSink>>  SinkHolder;

	private:
		//SinkHolder standardAssignedSinks;
		static Container cont;

	public:
		Logger& get(std::string const& loggerName = "default") {
			iterator find = cont.find(loggerName);
			if (find == cont.end()) {
				cont[loggerName] = std::make_unique<Logger>();				
				find = cont.find(loggerName);

				// Alternative way to do the same but no overwrite
				//auto result = cont.insert(std::make_pair(loggerName, std::make_unique<Logger>()));
				//find = result.first;
			}
			return *(find->second);
		}
		/*	
		void addStandardSink(LoggingSink& ls) {
			standardAssignedSinks.emplace_back(ls);
		}
		*/
			
	};
}
