cmake_minimum_required(VERSION 2.6)
PROJECT(AppendFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AppendFilter AppendFilter.cxx)
TARGET_LINK_LIBRARIES(AppendFilter vtkHybrid)

