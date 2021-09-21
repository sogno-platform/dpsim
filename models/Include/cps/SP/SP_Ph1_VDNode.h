/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cps/SimPowerComp.h>

namespace CPS {
namespace SP {
namespace Ph1 {

class VDNode : public SimPowerComp<Complex>, public SharedFactory<VDNode>{
    private:
        Real mDeltaSetPoint;
        Real mVoltageSetPointPerUnit;

    public:
        VDNode(String uid, String name,
            Logger::Level logLevel = Logger::Level::off);

	    VDNode(String uid, String name, Real vSetPointPerUnit,
		    Logger::Level logLevel = Logger::Level::off);

	    VDNode(String uid, String name, Real vSetPointPerUnit, Real delta,
		    Logger::Level logLevel = Logger::Level::off);


    };

}
}
}
