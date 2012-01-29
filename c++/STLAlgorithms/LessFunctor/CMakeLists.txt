cmake_minimum_required(VERSION 2.6)

PROJECT(LessFunctor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(LessFunctor LessFunctor.cpp)
