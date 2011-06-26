cmake_minimum_required(VERSION 2.6)
PROJECT(GlyphTable)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GlyphTable GlyphTable.cxx)
TARGET_LINK_LIBRARIES(GlyphTable vtkHybrid)

