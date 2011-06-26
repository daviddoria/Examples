cmake_minimum_required(VERSION 2.6)

PROJECT(OffScreenRendering)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OffScreenRendering OffScreenRendering.cxx)
TARGET_LINK_LIBRARIES(OffScreenRendering vtkHybrid)
