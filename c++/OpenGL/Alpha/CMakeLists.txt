cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLAlpha)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/GL/)
			
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(Alpha Alpha.cpp)

TARGET_LINK_LIBRARIES(Alpha glut GLU GL Xmu X11)
