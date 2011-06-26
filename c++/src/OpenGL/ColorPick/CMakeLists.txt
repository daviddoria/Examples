cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/GL/)
			
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(ColorPick ColorPick.cpp
MouseClass.h )

TARGET_LINK_LIBRARIES(ColorPick glut GLU GL Xmu X11)
