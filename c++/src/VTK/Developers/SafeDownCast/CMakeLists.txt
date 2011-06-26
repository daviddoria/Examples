cmake_minimum_required(VERSION 2.6)
PROJECT(SafeDownCastExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SafeDownCastExample SafeDownCastExample.cxx vtkPersonClass.cxx vtkLawyerClass.cxx)
TARGET_LINK_LIBRARIES(SafeDownCastExample vtkHybrid)

