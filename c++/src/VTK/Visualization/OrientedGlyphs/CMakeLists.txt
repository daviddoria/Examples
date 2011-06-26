cmake_minimum_required(VERSION 2.6)

PROJECT(OrientedGlyphs)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OrientedGlyphs OrientedGlyphs.cxx)
TARGET_LINK_LIBRARIES(OrientedGlyphs vtkHybrid)
