cmake_minimum_required(VERSION 2.6)
PROJECT(Timer)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Timer Timer.cxx)
TARGET_LINK_LIBRARIES(Timer vtkHybrid)

ADD_EXECUTABLE(Subclass Subclass.cxx)
TARGET_LINK_LIBRARIES(Subclass vtkHybrid)

