cmake_minimum_required(VERSION 2.6)

PROJECT(MaskPoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MaskPoints MaskPoints.cxx)
TARGET_LINK_LIBRARIES(MaskPoints vtkHybrid)
