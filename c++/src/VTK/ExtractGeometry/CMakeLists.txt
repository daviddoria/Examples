cmake_minimum_required(VERSION 2.6)

PROJECT(ExtractGeometry)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ExtractGeometry ExtractGeometry.cxx)
TARGET_LINK_LIBRARIES(ExtractGeometry vtkHybrid)
