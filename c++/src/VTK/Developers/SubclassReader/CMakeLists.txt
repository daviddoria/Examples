cmake_minimum_required(VERSION 2.6)
PROJECT(SubclassReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Example Example.cxx vtkTestReader.cxx)
TARGET_LINK_LIBRARIES(Example vtkHybrid)

