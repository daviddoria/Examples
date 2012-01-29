cmake_minimum_required(VERSION 2.6)

Project(Hash)

ADD_EXECUTABLE(Hash Hash.cpp)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")