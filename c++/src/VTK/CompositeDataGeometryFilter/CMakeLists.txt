cmake_minimum_required(VERSION 2.6)
PROJECT(CompositeDataGeometryFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CompositeDataGeometryFilter CompositeDataGeometryFilter.cxx)
TARGET_LINK_LIBRARIES(CompositeDataGeometryFilter vtkHybrid)

