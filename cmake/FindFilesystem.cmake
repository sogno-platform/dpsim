# Try to determine whether std::filesystem needs linking
# The test program exercises functions that require the runtime library to be
# linked in (a mere `#include <filesystem>` would compile fine even when the
# library is missing), so it lets us detect whether an extra -lstdc++fs /
# -lc++fs is actually required, or whether it's already part of libc / libc++
# (e.g. on macOS with Clang/libc++, or GCC >= 9).
include(CheckCXXSourceCompiles)

set(_filesystem_test_source "
	#include <filesystem>
	int main() {
		std::filesystem::path p{\".\"};
		return std::filesystem::exists(p) ? 0 : 1;
	}
")

set(CMAKE_REQUIRED_LIBRARIES "")
check_cxx_source_compiles("${_filesystem_test_source}" HAS_NATIVE_FILESYSTEM)

if(HAS_NATIVE_FILESYSTEM)
	# No extra library required
	set(FILESYSTEM_LIBRARY "")
elseif(NOT MSVC)
	# Retry linking against libstdc++fs (older GCC) or libc++fs (older Clang)
	foreach(_fs_lib stdc++fs c++fs)
		set(CMAKE_REQUIRED_LIBRARIES ${_fs_lib})
		check_cxx_source_compiles("${_filesystem_test_source}" HAS_FILESYSTEM_WITH_${_fs_lib})
		if(HAS_FILESYSTEM_WITH_${_fs_lib})
			set(FILESYSTEM_LIBRARY ${_fs_lib})
			set(HAS_NATIVE_FILESYSTEM TRUE)
			break()
		endif()
	endforeach()
	set(CMAKE_REQUIRED_LIBRARIES "")
endif()

if(NOT HAS_NATIVE_FILESYSTEM)
	# Not supported natively, suggest using ghc_filesystem
	message(WARNING "std::filesystem not supported, consider enabling FETCH_FILESYSTEM")
	set(FILESYSTEM_LIBRARY "")
endif()

set(FILESYSTEM_LIBRARIES ${FILESYSTEM_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Filesystem REQUIRED_VARS HAS_NATIVE_FILESYSTEM)

mark_as_advanced(FILESYSTEM_LIBRARY)

add_library(filesystem INTERFACE)

if(FILESYSTEM_LIBRARY)
	target_link_libraries(filesystem INTERFACE ${FILESYSTEM_LIBRARY})
endif()

# Export as standard alias if used via find_package(Filesystem)
add_library(Filesystem::filesystem ALIAS filesystem)
