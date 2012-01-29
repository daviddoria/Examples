cmake_minimum_required(VERSION 2.6)

Project(StaticConst)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(Working Working.cpp)

ADD_EXECUTABLE(StaticConst main.cpp StaticConst.cpp)
