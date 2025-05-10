# Try to determine whether std::filesystem needs linking
include(CheckCXXSourceCompiles)

check_cxx_source_compiles("
	#include <filesystem>
	int main() {
		std::filesystem::path p{\".\"};
		return 0;
	}
" HAS_NATIVE_FILESYSTEM)

# Determine link library, if needed
if(HAS_NATIVE_FILESYSTEM)
	if(MSVC)
		# MSVC has native support and doesn't require linking
		set(FILESYSTEM_LIBRARY "")
	else()
		# GCC < 9 or Clang may need explicit -lstdc++fs
		set(FILESYSTEM_LIBRARY stdc++fs)
	endif()
else()
	# Not supported natively, suggest using ghc_filesystem
	message(WARNING "std::filesystem not supported, consider enabling FETCH_FILESYSTEM")
	set(FILESYSTEM_LIBRARY "")
endif()

set(FILESYSTEM_LIBRARIES ${FILESYSTEM_LIBRARY})

include(FindPackageHandleStandardArgs)
if(NOT MSVC)
	find_package_handle_standard_args(Filesystem REQUIRED_VARS FILESYSTEM_LIBRARIES)
else()
	find_package_handle_standard_args(Filesystem DEFAULT_MSG)
endif()

mark_as_advanced(FILESYSTEM_LIBRARY)

add_library(filesystem INTERFACE)

if(FILESYSTEM_LIBRARY)
	target_link_libraries(filesystem INTERFACE ${FILESYSTEM_LIBRARY})
endif()

# Export as standard alias if used via find_package(Filesystem)
add_library(Filesystem::filesystem ALIAS filesystem)
