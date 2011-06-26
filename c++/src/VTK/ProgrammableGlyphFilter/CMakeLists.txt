cmake_minimum_required(VERSION 2.6)
PROJECT(ProgrammableGlyphFilter3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProgrammableGlyphFilter ProgrammableGlyphFilter.cxx)
TARGET_LINK_LIBRARIES(ProgrammableGlyphFilter vtkHybrid)

