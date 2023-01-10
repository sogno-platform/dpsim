include(FetchContent)
FetchContent_Declare(cim-data
	GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/grid-data/cim-grid-data.git
	GIT_TAG        master
)

FetchContent_MakeAvailable(cim-data)

FetchContent_Declare(profile-data
	GIT_REPOSITORY https://git.rwth-aachen.de/acs/public/grid-data/grid-profiles.git
	GIT_TAG        master
)

FetchContent_MakeAvailable(profile-data)
