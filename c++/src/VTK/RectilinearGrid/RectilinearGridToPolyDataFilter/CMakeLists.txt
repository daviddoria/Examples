cmake_minimum_required(VERSION 2.6)

PROJECT(RectilinearGridToPolyDataFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RectilinearGridToPolyDataFilter RectilinearGridToPolyDataFilter.cxx)

TARGET_LINK_LIBRARIES(RectilinearGridToPolyDataFilter vtkHybrid)
