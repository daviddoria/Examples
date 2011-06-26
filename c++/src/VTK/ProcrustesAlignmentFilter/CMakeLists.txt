cmake_minimum_required(VERSION 2.6)

PROJECT(ProcrustesAlignmentFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProcrustesAlignmentFilter ProcrustesAlignmentFilter.cxx)

TARGET_LINK_LIBRARIES(ProcrustesAlignmentFilter vtkHybrid)
