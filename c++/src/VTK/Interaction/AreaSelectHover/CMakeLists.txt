cmake_minimum_required(VERSION 2.6)

PROJECT(AreaSelectHover)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AreaSelectHover AreaSelectHover.cxx)
TARGET_LINK_LIBRARIES(AreaSelectHover vtkHybrid)
