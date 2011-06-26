cmake_minimum_required(VERSION 2.6)

PROJECT(FindAllArrayNames)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FindAllArrayNames FindAllArrayNames.cxx)
TARGET_LINK_LIBRARIES(FindAllArrayNames vtkHybrid)
