/** Event system
 *
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#include <deque>
#include <queue>

#include <dpsim/Config.h>
#include <cps/Definitions.h>
#include <cps/Attribute.h>
#include <cps/Logger.h>
#include <cps/Base/Base_Ph1_Switch.h>
#include <cps/Base/Base_Ph3_Switch.h>
#include <cps/PtrFactory.h>

namespace DPsim {

	class EventComparator;
	class EventQueue;

	class Event {

		friend class EventComparator;
		friend class EventQueue;

	protected:
		CPS::Real mTime;

	public:
		using Ptr = std::shared_ptr<Event>;

		virtual void execute() = 0;

		Event(CPS::Real t) :
			mTime(t)
		{ }

		virtual ~Event() {}
	};

	class EventComparator {
	public:
		bool operator() (const Event::Ptr &l, const Event::Ptr &r) {
			return l->mTime > r->mTime;
		}
	};

	template<typename T>
	class AttributeEvent : public Event, public SharedFactory<AttributeEvent<T>> {
	protected:
		typename CPS::Attribute<T>::Ptr mAttribute;
		T mNewValue;

	public:
		AttributeEvent(CPS::Real t, typename CPS::Attribute<T>::Ptr attr, T val) :
			Event(t),
			mAttribute(attr),
			mNewValue(val)
		{ }

		void execute() {
			mAttribute->set(mNewValue);
		}
	};

	class SwitchEvent : public Event, public SharedFactory<SwitchEvent> {

	protected:
		std::shared_ptr<CPS::Base::Ph1::Switch> mSwitch;
		CPS::Bool mNewState;

	public:
		using SharedFactory<SwitchEvent>::make;

		SwitchEvent(CPS::Real t, std::shared_ptr<CPS::Base::Ph1::Switch> sw, CPS::Bool state) :
			Event(t),
			mSwitch(sw),
			mNewState(state)
		{ }

		void execute() {
			if (mNewState)
				mSwitch->close();
			else
				mSwitch->open();
		}
	};

	class SwitchEvent3Ph : public Event, public SharedFactory<SwitchEvent3Ph> {

	protected:
		std::shared_ptr<CPS::Base::Ph3::Switch> mSwitch;
		CPS::Bool mNewState;

	public:
		using SharedFactory<SwitchEvent3Ph>::make;

		SwitchEvent3Ph(CPS::Real t, std::shared_ptr<CPS::Base::Ph3::Switch> sw, CPS::Bool state) :
			Event(t),
			mSwitch(sw),
			mNewState(state)
		{ }

		void execute() {
			if (mNewState)
				mSwitch->closeSwitch();
			else
				mSwitch->openSwitch();
		}
	};


	class EventQueue {

	protected:
		std::priority_queue<Event::Ptr, std::deque<Event::Ptr>, EventComparator> mEvents;

	public:
		///
		void addEvent(Event::Ptr e);
		///
		void handleEvents(CPS::Real currentTime);
	};
}

