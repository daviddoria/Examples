cmake_minimum_required(VERSION 2.6)

Project(static_assert)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(static_assert static_assert.cpp)
