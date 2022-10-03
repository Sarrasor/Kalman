add_subdirectory(fuser_config)
add_subdirectory(kalman_filter)
add_subdirectory(measurement)
add_subdirectory(object)

add_library(fuser 
	${CMAKE_CURRENT_LIST_DIR}/fuser.cpp	
	${CMAKE_CURRENT_LIST_DIR}/fuser.hpp	
)

target_include_directories(fuser
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(fuser 
	PUBLIC ${CONAN_LIBS} fuser_config measurement object
	PRIVATE kalman_filter
)

# Fuser test
add_executable(fuser_test
  ${CMAKE_CURRENT_LIST_DIR}/fuser_test.cpp
)

target_link_libraries(
  fuser_test
  PUBLIC fuser GTest::gtest_main
)

target_include_directories(fuser_test
	PRIVATE ${PROJECT_SOURCE_DIR}
)

gtest_discover_tests(fuser_test)

