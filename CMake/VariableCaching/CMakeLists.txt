cmake_minimum_required(VERSION 2.6)

PROJECT(VariableCaching)

#SET(CMAKE_CXX_FLAGS "Flags1" CACHE STRING "Compiler flags")
SET(CMAKE_CXX_FLAGS "Flags1")
message("Using flags ${CMAKE_CXX_FLAGS}")
ADD_EXECUTABLE(Executable1 VariableCaching.cxx)

#SET(CMAKE_CXX_FLAGS "Flags2" CACHE STRING "Compiler flags")
SET(CMAKE_CXX_FLAGS "Flags2")
message("Using flags ${CMAKE_CXX_FLAGS}")
ADD_EXECUTABLE(Executable2 VariableCaching.cxx)

