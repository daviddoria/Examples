cmake_minimum_required(VERSION 2.6)

PROJECT(CMakeDemo)
set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} /usr/include/GL/)
			
LINK_DIRECTORIES(${LINK_DIRECTORIES} /usr/lib)

ADD_EXECUTABLE(CMakeDemo cmake.cpp)

#TARGET_LINK_LIBRARIES(CMakeDemo Lib1 Lib2)

#can get help with the following command:
# cmake --help-command COMMAND
#i.e.
#cmake --help-command ADD_EXECUTABLE