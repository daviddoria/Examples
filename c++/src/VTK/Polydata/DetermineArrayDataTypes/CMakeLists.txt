cmake_minimum_required(VERSION 2.6)

PROJECT(DetermineArrayDataTypes)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DetermineArrayDataTypes DetermineArrayDataTypes.cxx)
TARGET_LINK_LIBRARIES(DetermineArrayDataTypes vtkHybrid)
