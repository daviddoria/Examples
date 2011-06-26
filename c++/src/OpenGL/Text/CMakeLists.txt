cmake_minimum_required(VERSION 2.6)

PROJECT(Text)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(Text Text.cpp)

TARGET_LINK_LIBRARIES(Text glut GLU GL Xmu X11 )
