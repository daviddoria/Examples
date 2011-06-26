cmake_minimum_required(VERSION 2.6)

PROJECT(ImageDataToStructuredGrid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageDataToStructuredGrid ImageDataToStructuredGrid.cxx)
TARGET_LINK_LIBRARIES(ImageDataToStructuredGrid vtkHybrid)
