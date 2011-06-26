cmake_minimum_required(VERSION 2.6)
PROJECT(ErrorMacro)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ErrorMacro ErrorMacro.cxx)
TARGET_LINK_LIBRARIES(ErrorMacro vtkHybrid)

