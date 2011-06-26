cmake_minimum_required(VERSION 2.6)

PROJECT(2DArray)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

ADD_EXECUTABLE(2DArray
2DArray.cpp

)

