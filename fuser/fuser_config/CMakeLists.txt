add_library(fuser_config 
	${CMAKE_CURRENT_LIST_DIR}/fuser_config.cpp	
	${CMAKE_CURRENT_LIST_DIR}/fuser_config.hpp	
)

target_include_directories(fuser_config
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(fuser_config 
	PUBLIC ${CONAN_LIBS} nlohmann_json::nlohmann_json
)

# Fuser Config test
add_executable(fuser_config_test
  ${CMAKE_CURRENT_LIST_DIR}/fuser_config_test.cpp
)

target_link_libraries(
  fuser_config_test
  PUBLIC fuser_config GTest::gtest_main
)

target_include_directories(fuser_config_test
	PRIVATE ${PROJECT_SOURCE_DIR}
)

gtest_discover_tests(fuser_config_test)

