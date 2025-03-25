/* Copyright 2017-2023 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#ifndef USE_GHC_FS
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif

#include <spdlog/fmt/ostr.h>

#if FMT_VERSION >= 90000
template <>
class fmt::formatter<fs::path>
    : public fmt::ostream_formatter {};
#endif
