cmake_minimum_required(VERSION 2.6)
PROJECT(ReverseSense)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReverseSense ReverseSense.cxx)
TARGET_LINK_LIBRARIES(ReverseSense vtkHybrid)

