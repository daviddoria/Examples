cmake_minimum_required(VERSION 2.6)

PROJECT(VectorMinMax)

ADD_EXECUTABLE(VectorMinMax VectorMinMax.cpp )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
