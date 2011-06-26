cmake_minimum_required(VERSION 2.6)
PROJECT(Normalize)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Normalize Normalize.cxx)
TARGET_LINK_LIBRARIES(Normalize vtkHybrid)

