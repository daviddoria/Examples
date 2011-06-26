cmake_minimum_required(VERSION 2.6)

PROJECT(FilterProgress)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FilterProgress FilterProgress.cxx)
TARGET_LINK_LIBRARIES(FilterProgress vtkHybrid)
