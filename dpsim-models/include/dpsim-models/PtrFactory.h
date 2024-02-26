/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <memory>
#include <utility>

/// Curiously recurring template pattern (CRTP) to create create new shared_ptr instances.
/// See: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
template <typename T> class SharedFactory {

public:
  template <typename... Args> static std::shared_ptr<T> make(Args &&...args) {
    return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
  }
};

template <typename T> class UniqueFactory {

public:
  template <typename... Args> static std::unique_ptr<T> make(Args &&...args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
};
