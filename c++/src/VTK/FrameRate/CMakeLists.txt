cmake_minimum_required(VERSION 2.6)

PROJECT(FrameRate)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FrameRate FrameRate.cxx)

TARGET_LINK_LIBRARIES(FrameRate vtkHybrid)
