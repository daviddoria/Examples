cmake_minimum_required(VERSION 2.6)
PROJECT(GetSetVector3Macro)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GetSetVector3Macro GetSetVector3Macro.cxx vtkTestClass.cxx)
TARGET_LINK_LIBRARIES(GetSetVector3Macro vtkHybrid)

