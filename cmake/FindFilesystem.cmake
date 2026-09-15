# SPDX-License-Identifier: MPL-2.0

include(CheckCXXSourceCompiles)
include(FindPackageHandleStandardArgs)

set(_FILESYSTEM_TEST_SOURCE "
	#include <filesystem>

	int main() {
		std::filesystem::path p{\".\"};
		return p.empty();
	}
")

# First try std::filesystem without any additional library.
# This is the normal case for:
#   - modern GCC
#   - modern Clang / AppleClang
#   - modern MSVC
set(CMAKE_REQUIRED_LIBRARIES "")
check_cxx_source_compiles(
	"${_FILESYSTEM_TEST_SOURCE}"
	FILESYSTEM_HAS_NATIVE_SUPPORT
)

if(FILESYSTEM_HAS_NATIVE_SUPPORT)

	set(FILESYSTEM_LIBRARY "")
	set(FILESYSTEM_FOUND TRUE)

else()

	# GCC versions before GCC 9 may require -lstdc++fs.
	set(CMAKE_REQUIRED_LIBRARIES stdc++fs)

	check_cxx_source_compiles(
		"${_FILESYSTEM_TEST_SOURCE}"
		FILESYSTEM_HAS_STDCXXFS
	)

	if(FILESYSTEM_HAS_STDCXXFS)

		set(FILESYSTEM_LIBRARY stdc++fs)
		set(FILESYSTEM_FOUND TRUE)

	else()

		# Older libc++ / Clang versions may require -lc++fs.
		set(CMAKE_REQUIRED_LIBRARIES c++fs)

		check_cxx_source_compiles(
			"${_FILESYSTEM_TEST_SOURCE}"
			FILESYSTEM_HAS_CXXFS
		)

		if(FILESYSTEM_HAS_CXXFS)

			set(FILESYSTEM_LIBRARY c++fs)
			set(FILESYSTEM_FOUND TRUE)

		else()

			set(FILESYSTEM_LIBRARY "")
			set(FILESYSTEM_FOUND FALSE)

		endif()

	endif()

endif()

# Do not leak test libraries into subsequent CMake checks.
set(CMAKE_REQUIRED_LIBRARIES "")

set(FILESYSTEM_LIBRARIES ${FILESYSTEM_LIBRARY})

if(FILESYSTEM_FOUND)

	if(FILESYSTEM_LIBRARY)
		message(STATUS
			"std::filesystem requires additional library: ${FILESYSTEM_LIBRARY}"
		)
	else()
		message(STATUS
			"std::filesystem is provided natively by the C++ standard library"
		)
	endif()

else()

	message(WARNING
		"std::filesystem is not available with the current compiler and standard library"
	)

endif()

find_package_handle_standard_args(
	Filesystem
	REQUIRED_VARS FILESYSTEM_FOUND
)

mark_as_advanced(
	FILESYSTEM_LIBRARY
)

add_library(filesystem INTERFACE)

if(FILESYSTEM_LIBRARY)
	target_link_libraries(
		filesystem
		INTERFACE
			${FILESYSTEM_LIBRARY}
	)
endif()

add_library(
	Filesystem::filesystem
	ALIAS
	filesystem
)
