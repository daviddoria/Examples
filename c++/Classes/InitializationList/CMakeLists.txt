cmake_minimum_required(VERSION 2.6)

Project(DefaultMemberInit)

ADD_EXECUTABLE(DefaultMemberInit DefaultMemberInit.cpp)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")