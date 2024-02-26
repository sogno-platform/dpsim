#!/usr/bin/env bash
#
# Format all C++ files in repo
#
# Author: Steffen Vogel <post@steffenvogel.de>
# SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

TOP_DIR=$(git rev-parse --show-toplevel)

find ${TOP_DIR} -iname "*.cpp" -o -iname "*.hpp" -o -iname "*.h" -o -iname "*.c" -a -not -path ./build | \
    xargs clang-format --verbose -i
