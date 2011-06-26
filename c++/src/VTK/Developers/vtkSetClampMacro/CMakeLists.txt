cmake_minimum_required(VERSION 2.6)
PROJECT(SetClampMacroExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SetClampMacroExample SetClampMacroExample.cxx vtkTestClass.cxx)
TARGET_LINK_LIBRARIES(SetClampMacroExample vtkHybrid)

