cmake_minimum_required(VERSION 2.6)

PROJECT(OggTheora)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OggTheora OggTheora.cxx)
TARGET_LINK_LIBRARIES(OggTheora vtkHybrid vtkIO)
