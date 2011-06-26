cmake_minimum_required(VERSION 2.6)

PROJECT(StructuredPointsReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredPointsReader StructuredPointsReader.cxx)
TARGET_LINK_LIBRARIES(StructuredPointsReader vtkHybrid)
