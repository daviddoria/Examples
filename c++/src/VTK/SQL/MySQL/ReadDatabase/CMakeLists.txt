cmake_minimum_required(VERSION 2.6)

PROJECT(ReadDatabase)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadDatabase ReadDatabase.cxx)
TARGET_LINK_LIBRARIES(ReadDatabase vtkHybrid)
