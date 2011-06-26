cmake_minimum_required(VERSION 2.6)
PROJECT(GlyphScaling)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GlyphScaling GlyphScaling.cxx)
TARGET_LINK_LIBRARIES(GlyphScaling vtkHybrid)

