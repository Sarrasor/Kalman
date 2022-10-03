add_library(kalman_filter 
	${CMAKE_CURRENT_LIST_DIR}/kalman_filter.cpp	
	${CMAKE_CURRENT_LIST_DIR}/kalman_filter.hpp	
)

target_include_directories(kalman_filter
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(kalman_filter 
	PUBLIC ${CONAN_LIBS}
)

# Kalman_filter test
add_executable(kalman_filter_test
  ${CMAKE_CURRENT_LIST_DIR}/kalman_filter_test.cpp
)

target_link_libraries(
  kalman_filter_test
  PUBLIC kalman_filter GTest::gtest_main
)

target_include_directories(kalman_filter_test
	PRIVATE ${PROJECT_SOURCE_DIR}
)

gtest_discover_tests(kalman_filter_test)

