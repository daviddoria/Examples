cmake_minimum_required(VERSION 2.6)

PROJECT(Progress)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Progress Progress.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(Progress vtkHybrid)
