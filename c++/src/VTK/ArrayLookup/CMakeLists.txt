cmake_minimum_required(VERSION 2.6)

PROJECT(ArrayLookup)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ArrayLookup ArrayLookup.cxx)
TARGET_LINK_LIBRARIES(ArrayLookup vtkHybrid)

