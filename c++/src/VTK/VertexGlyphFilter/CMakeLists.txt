cmake_minimum_required(VERSION 2.6)
PROJECT(VertexGlyphFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VertexGlyphFilter VertexGlyphFilter.cxx)
TARGET_LINK_LIBRARIES(VertexGlyphFilter vtkHybrid)

