cmake_minimum_required(VERSION 2.6)
PROJECT(TensorGlyph)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TensorGlyph TensorGlyph.cxx)
TARGET_LINK_LIBRARIES(TensorGlyph vtkHybrid)

