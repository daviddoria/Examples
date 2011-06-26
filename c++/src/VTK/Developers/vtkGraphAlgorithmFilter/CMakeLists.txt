cmake_minimum_required(VERSION 2.6)
PROJECT(vtkGraphAlgorithmFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FilterExample FilterExample.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(FilterExample vtkHybrid)

