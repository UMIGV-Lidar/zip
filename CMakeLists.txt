cmake_minimum_required(VERSION 2.8.3)

# c++14 support by doing this
# don't rely on compiler-specific extensions
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++14)

project(zip)

find_package(catkin REQUIRED
	COMPONENTS
		rosconsole
		roscpp
)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(zip2_test src/zip2_test.cpp)
target_link_libraries(zip2_test ${catkin_LIBRARIES})

install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)