cmake_minimum_required(VERSION 2.6)

PROJECT(Regex)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

ADD_EXECUTABLE(Regex Regex.cpp )
#target_link_libraries(Regex regex)

