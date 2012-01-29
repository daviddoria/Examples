cmake_minimum_required(VERSION 2.6)

Project(DynamicProperty)

set(Boost_USE_MULTITHREADED ON) # which is the default
FIND_PACKAGE(Boost 1.38 COMPONENTS)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(DynamicProperty DynamicProperty.cpp)
