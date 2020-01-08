/** Signalling part of a decoupling transmission line
 *
 * @file
 * @author Georg Reinke <georg.reinke@rwth-aachen.de>
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

#include <cps/DP/DP_Ph1_CurrentSource.h>
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/PowerComponent.h>
#include <cps/SignalComponent.h>
#include <cps/Task.h>

namespace CPS {
namespace Signal {
	class DecouplingLine :
		public SignalComponent,
		public SharedFactory<DecouplingLine> {
	protected:
		Real mDelay;
		Real mResistance;
		Real mInductance, mCapacitance;
		Real mSurgeImpedance;
		Complex mSrcCur1Ref;
		Complex mSrcCur2Ref;

		std::shared_ptr<DP::Node> mNode1, mNode2;
		std::shared_ptr<DP::Ph1::Resistor> mRes1, mRes2;
		std::shared_ptr<DP::Ph1::CurrentSource> mSrc1, mSrc2;
		Attribute<Complex>::Ptr mSrcCur1, mSrcCur2;

		// Ringbuffers for the values of previous timesteps
		// TODO make these matrix attributes
		std::vector<Complex> mVolt1, mVolt2, mCur1, mCur2;
		// workaround for dependency analysis as long as the states aren't attributes
		Matrix mStates;
		UInt mBufIdx = 0;
		UInt mBufSize;
		Real mAlpha;

		Complex interpolate(std::vector<Complex>& data);
	public:
		typedef std::shared_ptr<DecouplingLine> Ptr;

		DecouplingLine(String name, Node<Complex>::Ptr node1, Node<Complex>::Ptr node2,
			Real resistance, Real inductance, Real capacitance, Logger::Level logLevel = Logger::Level::info);

		DecouplingLine(String name, Logger::Level logLevel = Logger::Level::info);

		void setParameters(Node<Complex>::Ptr node1, Node<Complex>::Ptr node2, Real resistance, Real inductance, Real capacitance);
		void initialize(Real omega, Real timeStep);
		void step(Real time, Int timeStepCount);
		void postStep();
		Task::List getTasks();
		Component::List getLineComponents();

		class PreStep : public Task {
		public:
			PreStep(DecouplingLine& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mPrevStepDependencies.push_back(mLine.attribute("states"));
				mModifiedAttributes.push_back(mLine.mSrc1->attribute("I_ref"));
				mModifiedAttributes.push_back(mLine.mSrc2->attribute("I_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			DecouplingLine& mLine;
		};

		class PostStep : public Task {
		public:
			PostStep(DecouplingLine& line) :
				Task(line.mName + ".PostStep"), mLine(line) {
				mAttributeDependencies.push_back(mLine.mRes1->attribute("v_intf"));
				mAttributeDependencies.push_back(mLine.mRes1->attribute("i_intf"));
				mAttributeDependencies.push_back(mLine.mRes2->attribute("v_intf"));
				mAttributeDependencies.push_back(mLine.mRes2->attribute("i_intf"));
				mModifiedAttributes.push_back(mLine.attribute("states"));
				}

			void execute(Real time, Int timeStepCount);

		private:
			DecouplingLine& mLine;
		};
	};
}
}
