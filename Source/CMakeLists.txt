set(SOURCES
	Simulation.cpp
	RealTimeSimulation.cpp
	MNASolver.cpp
	Utils.cpp
	Timer.cpp
	Event.cpp
	DataLogger.cpp
)

list(APPEND LIBRARIES cps)

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

if(WITH_RT AND HAVE_TIMERFD)
	list(APPEND LIBRARIES "-lrt")
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

if(WITH_SHMEM)
	list(APPEND SOURCES
		Interface.cpp
	)

	list(APPEND LIBRARIES ${VILLASNODE_LIBRARIES})
	list(APPEND INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS})
endif()

add_library(dpsim)
target_sources(dpsim PRIVATE ${SOURCES})
target_link_libraries(dpsim ${LIBRARIES})
target_include_directories(dpsim PUBLIC ${INCLUDE_DIRS})
target_compile_options(dpsim PUBLIC ${DPSIM_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

install(TARGETS dpsim
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
)
