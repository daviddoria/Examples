cmake_minimum_required(VERSION 2.6)

PROJECT(StructuredGridReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(StructuredGridReader StructuredGridReader.cxx)
TARGET_LINK_LIBRARIES(StructuredGridReader vtkHybrid)
