message(STATUS "Searching for Boost")
find_package(Boost 1.74.0 COMPONENTS program_options REQUIRED)
if(Boost_FOUND)
    message(STATUS "Found Boost")
else()
    message (FATAL_ERROR "Cannot find Boost")
endif()

message(STATUS "Fetching Gtest")
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

message(STATUS "Adding Eigen via Conan")
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()