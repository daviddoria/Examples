cmake_minimum_required(VERSION 2.6)
PROJECT(Glyph3DMapper)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Glyph3DMapper Glyph3DMapper.cxx)
TARGET_LINK_LIBRARIES(Glyph3DMapper vtkHybrid)

