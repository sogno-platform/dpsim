/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <deque>
#include <queue>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/Base/Base_Ph3_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim/Config.h>

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

  Event(CPS::Real t) : mTime(t) {}

  virtual ~Event() {}
};

class EventComparator {
public:
  bool operator()(const Event::Ptr &l, const Event::Ptr &r) {
    return l->mTime > r->mTime;
  }
};

template <typename T>
class AttributeEvent : public Event, public SharedFactory<AttributeEvent<T>> {
protected:
  typename CPS::Attribute<T>::Ptr mAttribute;
  T mNewValue;

public:
  AttributeEvent(CPS::Real t, typename CPS::Attribute<T>::Ptr attr, T val)
      : Event(t), mAttribute(attr), mNewValue(val) {}

  void execute() { mAttribute->set(mNewValue); }
};

class SwitchEvent : public Event, public SharedFactory<SwitchEvent> {

protected:
  std::shared_ptr<CPS::Base::Ph1::Switch> mSwitch;
  CPS::Bool mNewState;

public:
  using SharedFactory<SwitchEvent>::make;

  SwitchEvent(CPS::Real t, const std::shared_ptr<CPS::Base::Ph1::Switch> &sw,
              CPS::Bool state)
      : Event(t), mSwitch(sw), mNewState(state) {}

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

  SwitchEvent3Ph(CPS::Real t, const std::shared_ptr<CPS::Base::Ph3::Switch> &sw,
                 CPS::Bool state)
      : Event(t), mSwitch(sw), mNewState(state) {}

  void execute() {
    if (mNewState)
      mSwitch->closeSwitch();
    else
      mSwitch->openSwitch();
  }
};

class EventQueue {

protected:
  std::priority_queue<Event::Ptr, std::deque<Event::Ptr>, EventComparator>
      mEvents;

public:
  ///
  void addEvent(Event::Ptr e);
  ///
  void handleEvents(CPS::Real currentTime);
};
} // namespace DPsim
