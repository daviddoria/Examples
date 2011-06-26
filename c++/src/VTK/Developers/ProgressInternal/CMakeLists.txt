cmake_minimum_required(VERSION 2.6)

PROJECT(ProgressInternal)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProgressInternal ProgressInternal.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(ProgressInternal vtkHybrid)
