/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <cps/DP/DP_Ph1_CurrentSource.h>
#include <cps/DP/DP_Ph1_Resistor.h>
#include <cps/SimPowerComp.h>
#include <cps/SimSignalComp.h>
#include <cps/Task.h>

namespace CPS {
namespace Signal {
	class DecouplingLine :
		public SimSignalComp,
		public SharedFactory<DecouplingLine> {
	protected:
		Real mDelay;
		Real mResistance;
		Real mInductance, mCapacitance;
		Real mSurgeImpedance;
		Complex mSrcCur1Ref;
		Complex mSrcCur2Ref;

		std::shared_ptr<DP::SimNode> mNode1, mNode2;
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

		DecouplingLine(String name, SimNode<Complex>::Ptr node1, SimNode<Complex>::Ptr node2,
			Real resistance, Real inductance, Real capacitance, Logger::Level logLevel = Logger::Level::info);

		DecouplingLine(String name, Logger::Level logLevel = Logger::Level::info);

		void setParameters(SimNode<Complex>::Ptr node1, SimNode<Complex>::Ptr node2, Real resistance, Real inductance, Real capacitance);
		void initialize(Real omega, Real timeStep);
		void step(Real time, Int timeStepCount);
		void postStep();
		Task::List getTasks();
		IdentifiedObject::List getLineComponents();

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
