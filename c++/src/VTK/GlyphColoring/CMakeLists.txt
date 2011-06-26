cmake_minimum_required(VERSION 2.6)
PROJECT(GlyphColoring)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GlyphColoring GlyphColoring.cxx)
TARGET_LINK_LIBRARIES(GlyphColoring vtkHybrid)

