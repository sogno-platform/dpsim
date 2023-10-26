include(FetchContent)
FetchContent_Declare(cim-data
	GIT_REPOSITORY https://github.com/dpsim-simulator/cim-grid-data.git
	GIT_TAG        master
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(cim-data)

FetchContent_Declare(profile-data
	GIT_REPOSITORY https://github.com/dpsim-simulator/example-profile-data.git
	GIT_TAG        master
	GIT_SHALLOW    TRUE
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(profile-data)
