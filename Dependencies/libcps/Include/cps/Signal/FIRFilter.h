/**
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
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

#include <vector>

#include <cps/PowerComponent.h>
#include <cps/SignalComponent.h>
#include <cps/Task.h>

namespace CPS {
namespace Signal {
	class FIRFilter :
		public SignalComponent,
		public SharedFactory<FIRFilter> {
	protected:
		std::vector<Real> mSignal;
		std::vector<Real> mFilter;
		Real mOutput;
		Attribute<Real>::Ptr mInput;
		Int mCurrentIdx;
		Int mFilterLength;
		Real mInitSample;

		void incrementIndex();
		Int getIndex(Int index);
	public:
		FIRFilter(String uid, String name, Logger::Level logLevel = Logger::Level::off);
		FIRFilter(String name, std::vector<Real> filterCoefficients, Real initSample = 1, Logger::Level logLevel = Logger::Level::off);

		void initialize(Real timeStep);
		void step(Real time);
		void setInput(Attribute<Real>::Ptr input);
		Task::List getTasks();

		class Step : public Task {
		public:
			Step(FIRFilter& filter) :
				Task(filter.mName + ".Step"), mFilter(filter) {
				mAttributeDependencies.push_back(filter.mInput);
				mModifiedAttributes.push_back(filter.attribute("output"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			FIRFilter& mFilter;
		};
	};
}
}
