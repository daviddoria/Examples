cmake_minimum_required(VERSION 2.6)
PROJECT(ImageDataGeometryFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageDataGeometryFilter ImageDataGeometryFilter.cxx)
TARGET_LINK_LIBRARIES(ImageDataGeometryFilter vtkHybrid)

