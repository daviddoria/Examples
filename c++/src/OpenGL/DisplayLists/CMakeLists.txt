cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLDisplayLists)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(Test DisplayLists.cpp  )

TARGET_LINK_LIBRARIES(Test glut GLU GL Xmu X11)
