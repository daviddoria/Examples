cmake_minimum_required(VERSION 2.6)

PROJECT(RibbonFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RibbonFilter RibbonFilter.cxx)
TARGET_LINK_LIBRARIES(RibbonFilter vtkHybrid)

