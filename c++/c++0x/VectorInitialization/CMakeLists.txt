cmake_minimum_required(VERSION 2.6)

PROJECT(VectorInitialization)

ADD_EXECUTABLE(VectorInitialization VectorInitialization.cpp )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
