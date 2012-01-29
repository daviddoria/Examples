cmake_minimum_required(VERSION 2.6)

Project(DistanceBetweenPoints)

set(Boost_USE_MULTITHREADED ON)
#FIND_PACKAGE(Boost 1.38 COMPONENTS required)
#FIND_PACKAGE(Boost required)
FIND_PACKAGE(Boost)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(DistanceBetweenPoints DistanceBetweenPoints.cpp)
