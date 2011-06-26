cmake_minimum_required(VERSION 2.6)

PROJECT(Mouse)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/GL/)
			
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(Mouse Mouse.cpp)

TARGET_LINK_LIBRARIES(Mouse glut GLU GL Xmu X11)
