cmake_minimum_required(VERSION 3.10)

project (
	kalman 
	LANGUAGES CXX
)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/AddCompilerFlags.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/AddLibraries.cmake)

enable_testing()
include(GoogleTest)

add_subdirectory(third_party)
add_subdirectory(fuser)
add_subdirectory(apps)