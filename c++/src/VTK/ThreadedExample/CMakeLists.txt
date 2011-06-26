cmake_minimum_required(VERSION 2.6)

PROJECT(ThreadedExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ThreadedExample ThreadedExample.cxx)

TARGET_LINK_LIBRARIES(ThreadedExample vtkHybrid)
