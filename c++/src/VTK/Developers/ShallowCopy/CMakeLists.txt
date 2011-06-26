cmake_minimum_required(VERSION 2.6)
PROJECT(ShallowCopyExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShallowCopyExample ShallowCopyExample.cxx vtkPersonClass.cxx vtkLawyerClass.cxx)
TARGET_LINK_LIBRARIES(ShallowCopyExample vtkHybrid)

