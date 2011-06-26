cmake_minimum_required(VERSION 2.6)

PROJECT(ScalarBarActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ScalarBarActor ScalarBarActor.cxx)
TARGET_LINK_LIBRARIES(ScalarBarActor vtkHybrid)
