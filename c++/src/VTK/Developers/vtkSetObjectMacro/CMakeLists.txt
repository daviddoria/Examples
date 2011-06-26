cmake_minimum_required(VERSION 2.6)
PROJECT(SetObjectMacroExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SetObjectMacroExample SetObjectMacroExample.cxx vtkTestClass.cxx)
TARGET_LINK_LIBRARIES(SetObjectMacroExample vtkHybrid)

