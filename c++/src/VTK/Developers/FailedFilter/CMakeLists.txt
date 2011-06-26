cmake_minimum_required(VERSION 2.6)
PROJECT(FailedFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FailedFilter FailedFilter.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(FailedFilter vtkHybrid)

