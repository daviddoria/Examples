cmake_minimum_required(VERSION 2.6)

PROJECT(ExtractComponents)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractComponents ExtractComponents.cxx)
TARGET_LINK_LIBRARIES(ExtractComponents vtkHybrid)

