cmake_minimum_required(VERSION 2.6)

PROJECT(Sort)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(Sort Sort.cpp )

ADD_EXECUTABLE(SortSmartPointers SortSmartPointers.cpp)

ADD_EXECUTABLE(SortPoint SortPoint.cpp)
