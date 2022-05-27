/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimSignalComp.h>


namespace CPS {
namespace Signal {
	class SignalGenerator :
		public SimSignalComp {
    public:
		typedef std::shared_ptr<SignalGenerator> Ptr;
		typedef std::vector<Ptr> List;

		const CPS::Attribute<Complex>::Ptr mSigOut;
		const CPS::Attribute<Real>::Ptr mFreq;

		SignalGenerator(String uid, String name, Logger::Level logLevel = Logger::Level::off);

		SignalGenerator(String name, Logger::Level logLevel = Logger::Level::off)
			: SignalGenerator(name, name, logLevel) { }

		/// updates current signal
        virtual void step(Real time) = 0;
		/// returns current signal value without updating it
		Complex getSignal();
		
		/// task not needed, update called by owning object
		/// TODO: think about changing that and implementing a task here
		/*
		Task::List getTasks();

        class Step : public Task {
		public:
			Step(SignalGenerator& sigGen) :
					Task(**sigGen.mName + ".Step"), mSigGen(sigGen) {
				mModifiedAttributes.push_back();
			}

			void execute(Real time, Int timeStepCount);

		private:
			SignalGenerator& mSigGen;
		};*/
    };
}
}
