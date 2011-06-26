cmake_minimum_required(VERSION 2.6)
PROJECT(DebugMacro)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DebugMacro DebugMacro.cxx vtkTestClass.cxx)
TARGET_LINK_LIBRARIES(DebugMacro vtkHybrid)

