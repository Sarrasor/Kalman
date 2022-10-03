add_executable(main 
	${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_include_directories(main
	PRIVATE ${PROJECT_SOURCE_DIR}
)

target_link_libraries(main 
	PUBLIC fuser ${Boost_LIBRARIES} nlohmann_json::nlohmann_json
)