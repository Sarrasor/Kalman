add_library(measurement 
	${CMAKE_CURRENT_LIST_DIR}/measurement.cpp	
	${CMAKE_CURRENT_LIST_DIR}/measurement.hpp	
)

target_include_directories(measurement
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(measurement 
	PUBLIC ${CONAN_LIBS} nlohmann_json::nlohmann_json
)

# Measurement test
add_executable(measurement_test
  ${CMAKE_CURRENT_LIST_DIR}/measurement_test.cpp
)

target_link_libraries(
  measurement_test
  PUBLIC measurement GTest::gtest_main
)

target_include_directories(measurement_test
	PRIVATE ${PROJECT_SOURCE_DIR}
)

gtest_discover_tests(measurement_test)

