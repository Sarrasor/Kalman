add_library(object 
	${CMAKE_CURRENT_LIST_DIR}/object.cpp	
	${CMAKE_CURRENT_LIST_DIR}/object.hpp	
)

target_include_directories(object
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(object 
	PUBLIC ${CONAN_LIBS} nlohmann_json::nlohmann_json
)

# Object test
add_executable(object_test
  ${CMAKE_CURRENT_LIST_DIR}/object_test.cpp
)

target_link_libraries(
  object_test
  PUBLIC object GTest::gtest_main
)

target_include_directories(object_test
	PRIVATE ${PROJECT_SOURCE_DIR}
)

gtest_discover_tests(object_test)

