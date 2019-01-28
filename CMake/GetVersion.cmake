# CMakeLists.txt.
#
# @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
# @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
# @license GNU General Public License (version 3)
#
# VILLASnode
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
###################################################################################

# This CMake function sets the following variables:
#
# ${PREFIX}_VERSION_STR       v0.6.3
# ${PREFIX}_VERSION           0.6.3
# ${PREFIX}_MAJOR_VERSION     0
# ${PREFIX}_MINOR_VERSION     6
# ${PREFIX}_PATCH_VERSION     3
# ${PREFIX}_RELEASE           1.ci_tests_release.20180823git1cd25c2f
# ${PREFIX}_GIT_REV           1cd25c2f
# ${PREFIX}_GIT_BRANCH        ci-tests
# ${PREFIX}_GIT_BRANCH_NORM   ci_tests
# ${PREFIX}_VARIANT           release
# ${PREFIX}_VARIANT_NORM      release
# ${PREFIX}_BUILD_ID          v0.6.3-1cd25c2f-release
# ${PREFIX}_BUILD_DATE        20180823

function(GetVersion DIR PREFIX)
    execute_process(
        COMMAND git describe --tags --abbrev=0 --match "v*"
        WORKING_DIRECTORY ${DIR}
        OUTPUT_VARIABLE VERSION_STR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE RC
    )

    if(NOT RC EQUAL 0)
        message(FATAL_ERROR
            "Failed to retrieve version information from Git. "
            "Make sure that the source directory is a Git repo and "
            "contains at least one valid tag like 'v0.1.0'"
        )
    endif()

    string(REGEX REPLACE "^v([0-9]+\\.[0-9]+\\.[0-9]+)$" "\\1"     VERSION       ${VERSION_STR})
    string(REGEX REPLACE "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\1" MAJOR_VERSION ${VERSION_STR})
    string(REGEX REPLACE "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\2" MINOR_VERSION ${VERSION_STR})
    string(REGEX REPLACE "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\3" PATCH_VERSION ${VERSION_STR})

    string(TIMESTAMP BUILD_DATE "%Y%m%d")

    if(CMAKE_BUILD_TYPE)
        string(TOLOWER "${CMAKE_BUILD_TYPE}" VARIANT)
    else()
        set(VARIANT "release")
    endif()

    if(DEFINED ENV{CI})
    	string(APPEND VARIANT "-ci")
        string(SUBSTRING $ENV{CI_COMMIT_SHA} 0 7 GIT_REV)
    	set(GIT_BRANCH $ENV{CI_COMMIT_REF_NAME})
    else()
        execute_process(
            COMMAND git rev-parse --short=7 HEAD
            WORKING_DIRECTORY ${DIR}
            OUTPUT_VARIABLE GIT_REV
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        execute_process(
            COMMAND git rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY ${DIR}
            OUTPUT_VARIABLE GIT_BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()

    if(DEFINED ENV{CI_COMMIT_TAG})
    	set(RELEASE 1)
    else()
        string(REPLACE "-" "_" GIT_BRANCH_NORM ${GIT_BRANCH})
        string(REPLACE "-" "_" VARIANT_NORM    ${VARIANT})

        set(RELEASE "1.${GIT_BRANCH_NORM}_${VARIANT_NORM}.${BUILD_DATE}git${GIT_REV}")
    endif()

    set(BUILD_ID "v${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}-${GIT_REV}-${VARIANT}" )

    # Return results to parent scope
    set(${PREFIX}_VERSION_STR     ${VERSION_STR}      PARENT_SCOPE)
    set(${PREFIX}_VERSION         ${VERSION}          PARENT_SCOPE)
    set(${PREFIX}_MAJOR_VERSION   ${MAJOR_VERSION}    PARENT_SCOPE)
    set(${PREFIX}_MINOR_VERSION   ${MINOR_VERSION}    PARENT_SCOPE)
    set(${PREFIX}_PATCH_VERSION   ${PATCH_VERSION}    PARENT_SCOPE)
    set(${PREFIX}_RELEASE         ${RELEASE}          PARENT_SCOPE)
    set(${PREFIX}_GIT_REV         ${GIT_REV}          PARENT_SCOPE)
    set(${PREFIX}_GIT_BRANCH      ${GIT_BRANCH}       PARENT_SCOPE)
    set(${PREFIX}_GIT_BRANCH_NORM ${GIT_BRANCH_NORM}  PARENT_SCOPE)
    set(${PREFIX}_VARIANT         ${VARIANT}          PARENT_SCOPE)
    set(${PREFIX}_VARIANT_NORM    ${VARIANT_NORM}     PARENT_SCOPE)
    set(${PREFIX}_BUILD_ID        ${BUILD_ID}         PARENT_SCOPE)
    set(${PREFIX}_BUILD_DATE      ${BUILD_DATE}       PARENT_SCOPE)
endfunction(GetVersion)
