cmake_minimum_required(VERSION 2.6)

PROJECT(AlignFrames)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AlignFrames AlignFrames.cxx)
TARGET_LINK_LIBRARIES(AlignFrames vtkHybrid)
