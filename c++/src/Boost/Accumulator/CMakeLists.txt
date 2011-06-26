cmake_minimum_required(VERSION 2.6)

Project(Accumulator)

#FIND_PACKAGE(Boost 1.38 COMPONENTS program_options required)

#INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
#LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(Accumulator Accumulator.cpp)

ADD_EXECUTABLE(Density Density.cpp)