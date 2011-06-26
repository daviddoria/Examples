cmake_minimum_required(VERSION 2.6)

Project(BundledPropertySet)

FIND_PACKAGE(Boost)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(BundledPropertySet BundledPropertySet.cpp)


