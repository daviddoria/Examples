cmake_minimum_required(VERSION 2.6)

Project(HamiltonianCircuit)

FIND_PACKAGE(Boost)

LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})

ADD_EXECUTABLE(HamiltonianCircuit HamiltonianCircuit.cpp)
