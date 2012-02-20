cmake_minimum_required(VERSION 2.6)

PROJECT(enable_if)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(enable_if enable_if.cpp )

ADD_EXECUTABLE(function function.cpp )

ADD_EXECUTABLE(voidMemberFunction voidMemberFunction.cpp )

#ADD_EXECUTABLE(operator operator.cpp )
