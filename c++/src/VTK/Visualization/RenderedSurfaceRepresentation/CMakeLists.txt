cmake_minimum_required(VERSION 2.6)

PROJECT(RenderedSurfaceRepresentation)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RenderedSurfaceRepresentation RenderedSurfaceRepresentation.cxx)
TARGET_LINK_LIBRARIES(RenderedSurfaceRepresentation vtkHybrid)

