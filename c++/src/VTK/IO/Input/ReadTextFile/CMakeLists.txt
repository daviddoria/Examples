cmake_minimum_required(VERSION 2.6)
PROJECT(ReadTextFile)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReadTextFile ReadTextFile.cxx)
TARGET_LINK_LIBRARIES(ReadTextFile vtkHybrid)

