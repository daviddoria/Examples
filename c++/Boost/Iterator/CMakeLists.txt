cmake_minimum_required(VERSION 2.6)

Project(BoostIterator)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

ADD_EXECUTABLE(Iterator Iterator.cpp)

ADD_EXECUTABLE(IndirectIterator IndirectIterator.cpp)

ADD_EXECUTABLE(IndirectIteratorSmartPointer IndirectIteratorSmartPointer.cpp)

ADD_EXECUTABLE(IndirectIteratorSmartPointerSorting IndirectIteratorSmartPointerSorting.cpp)
