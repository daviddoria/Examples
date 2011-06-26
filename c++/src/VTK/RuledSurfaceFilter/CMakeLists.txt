cmake_minimum_required(VERSION 2.6)
PROJECT(RuledSurfaceFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RuledSurfaceFilter RuledSurfaceFilter.cxx)
TARGET_LINK_LIBRARIES(RuledSurfaceFilter vtkHybrid)

