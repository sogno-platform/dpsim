/**
 * @copyright 2017 Institute for Automation of Complex Power Systems, EONERC
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

#include <cps/EMT/EMT_Ph1_CurrentSource.h>
#include <cps/EMT/EMT_Ph1_Resistor.h>
#include <cps/SignalComponent.h>
#include <cps/Task.h>

namespace CPS {
namespace Signal {
	class DecouplingLineEMT :
		public SignalComponent,
		public SharedFactory<DecouplingLineEMT> {
	protected:
		Real mDelay;
		Real mResistance;
		Real mInductance;
		Real mCapacitance;
		Real mSurgeImpedance;
		Real mSrcCur1Ref;
		Real mSrcCur2Ref;

		std::shared_ptr<EMT::Node> mNode1, mNode2;
		std::shared_ptr<EMT::Ph1::Resistor> mRes1, mRes2;
		std::shared_ptr<EMT::Ph1::CurrentSource> mSrc1, mSrc2;
		Attribute<Complex>::Ptr mSrcCur1, mSrcCur2;

		// Ringbuffers for the values of previous timesteps
		// TODO make these matrix attributes
		std::vector<Real> mVolt1, mVolt2, mCur1, mCur2;
		// workaround for dependency analysis as long as the states aren't attributes
		Matrix mStates;
		UInt mBufIdx = 0;
		UInt mBufSize;
		Real mAlpha;

		Real interpolate(std::vector<Real>& data);
	public:
		typedef std::shared_ptr<DecouplingLineEMT> Ptr;

		DecouplingLineEMT(String name, Logger::Level logLevel = Logger::Level::info);

		void setParameters(Node<Real>::Ptr node1, Node<Real>::Ptr node2,
			Real resistance, Real inductance, Real capacitance);
		void initialize(Real omega, Real timeStep);
		void step(Real time, Int timeStepCount);
		void postStep();
		Task::List getTasks();
		Component::List getLineComponents();

		class PreStep : public Task {
		public:
			PreStep(DecouplingLineEMT& line) :
				Task(line.mName + ".MnaPreStep"), mLine(line) {
				mPrevStepDependencies.push_back(mLine.attribute("states"));
				mModifiedAttributes.push_back(mLine.mSrc1->attribute("I_ref"));
				mModifiedAttributes.push_back(mLine.mSrc2->attribute("I_ref"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			DecouplingLineEMT& mLine;
		};

		class PostStep : public Task {
		public:
			PostStep(DecouplingLineEMT& line) :
				Task(line.mName + ".PostStep"), mLine(line) {
				mAttributeDependencies.push_back(mLine.mRes1->attribute("v_intf"));
				mAttributeDependencies.push_back(mLine.mRes1->attribute("i_intf"));
				mAttributeDependencies.push_back(mLine.mRes2->attribute("v_intf"));
				mAttributeDependencies.push_back(mLine.mRes2->attribute("i_intf"));
				mModifiedAttributes.push_back(mLine.attribute("states"));
			}

			void execute(Real time, Int timeStepCount);

		private:
			DecouplingLineEMT& mLine;
		};
	};
}
}
