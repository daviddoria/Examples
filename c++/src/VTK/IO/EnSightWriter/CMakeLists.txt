cmake_minimum_required(VERSION 2.6)

PROJECT(EnSightWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EnSightWriter EnSightWriter.cxx)
TARGET_LINK_LIBRARIES(EnSightWriter vtkHybrid)

