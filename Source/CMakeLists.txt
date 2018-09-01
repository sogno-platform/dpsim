set(SOURCES
	Simulation.cpp
	MNASolver.cpp
	Utils.cpp
)

set(LIBRARIES cps)

if(NOT MSVC)
	list(APPEND LIBRARIES "-lstdc++fs")
endif()

if(NOT WIN32)
	# we use PyArg_ParseTupleAndKeywords several times, which erroneously accepts
	# a char* array instead of a const char* array
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -fPIC")
endif()

if(WITH_RT)
	list(APPEND LIBRARIES "-lrt")
	list(APPEND SOURCES RealTimeSimulation.cpp)
endif()

if(WITH_SUNDIALS)
	list(APPEND SOURCES DAESolver.cpp)

	list(APPEND INCLUDE_DIRS ${SUNDIALS_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${SUNDIALS_LIBRARIES})
endif()

if(WITH_PYTHON)
	list(APPEND INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
	list(APPEND LIBRARIES ${PYTHON_LIBRARIES})

	add_subdirectory(Python)
endif()

add_library(dpsim)
target_sources(dpsim PRIVATE ${SOURCES})
target_link_libraries(dpsim ${LIBRARIES})
target_include_directories(dpsim PRIVATE ${INCLUDE_DIRS})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

install(TARGETS dpsim
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
)
