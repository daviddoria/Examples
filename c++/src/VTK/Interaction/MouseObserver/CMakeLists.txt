cmake_minimum_required(VERSION 2.6)

PROJECT(MouseObserver)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MouseObserver MouseObserver.cxx)
TARGET_LINK_LIBRARIES(MouseObserver vtkHybrid)

