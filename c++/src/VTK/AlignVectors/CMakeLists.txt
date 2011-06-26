cmake_minimum_required(VERSION 2.6)

PROJECT(AlignVectors)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AlignVectors AlignVectors.cxx)
TARGET_LINK_LIBRARIES(AlignVectors vtkHybrid)
