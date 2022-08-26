include(FetchContent)

FetchContent_Declare(
  readerwriterqueue
  GIT_REPOSITORY    https://github.com/cameron314/readerwriterqueue
  GIT_TAG           v1.0.6
)

# TODO: Is this really still necessary? Or can we just use cmake 3.14?
if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
  FetchContent_GetProperties(readerwriterqueue)
  if(NOT readerwriterqueue_POPULATED)
    FetchContent_Populate(readerwriterqueue)
    add_subdirectory(${readerwriterqueue_SOURCE_DIR} ${readerwriterqueue_BINARY_DIR})
  endif()
else()
  FetchContent_MakeAvailable(readerwriterqueue)
endif()