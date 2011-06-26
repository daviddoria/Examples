cmake_minimum_required(VERSION 2.6)

PROJECT(EigenValuesVectors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EigenValuesVectors EigenValuesVectors.cxx)

TARGET_LINK_LIBRARIES(EigenValuesVectors vtkHybrid)
