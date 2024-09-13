if (WIN32)
	message(STATUS "Sundials is currently not supported in Windows")
else()
	find_path(SUNDIALS_INCLUDE_DIR
		NAMES sundials_version.h
		PATH_SUFFIXES include
	)

	find_library(SUNDIALS_ARKODE_LIBRARY
		NAMES sundials_arkode
	)

	find_library(SUNDIALS_CVODE_LIBRARY
		NAMES sundials_cvode
	)

	find_library(SUNDIALS_CVODES_LIBRARY
		NAMES sundials_cvodes
	)

	find_library(SUNDIALS_IDA_LIBRARY
		NAMES sundials_ida
	)

	find_library(SUNDIALS_IDAS_LIBRARY
		NAMES sundials_idas
	)

	find_library(SUNDIALS_KINSOL_LIBRARY
		NAMES sundials_kinsol
	)

	find_library(SUNDIALS_NVECSERIAL_LIBRARY
		NAMES sundials_nvecserial
	)

	find_library(SUNDIALS_NVECOPENMP_LIBRARY
		NAMES sundials_nvecopenmp
	)

	find_library(SUNDIALS_NVECPTHREADS_LIBRARY
		NAMES sundials_nvecpthreads
	)

	set(SUNDIALS_LIBRARIES
		${SUNDIALS_ARKODE_LIBRARY}
		${SUNDIALS_CVODE_LIBRARY}
		${SUNDIALS_CVODES_LIBRARY}
		${SUNDIALS_IDA_LIBRARY}
		${SUNDIALS_IDAS_LIBRARY}
		${SUNDIALS_KINSOL_LIBRARY}
		${SUNDIALS_NVECSERIAL_LIBRARY}
	)

	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(Sundials DEFAULT_MSG SUNDIALS_LIBRARIES SUNDIALS_INCLUDE_DIR)

	mark_as_advanced(SUNDIALS_INCLUDE_DIR)
endif()
