cmake_minimum_required(VERSION 2.6)

PROJECT(SelectPixel)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectPixel SelectPixel.cxx VtkObserverMouseMove.cpp VtkObserverMouseClick.cpp)
TARGET_LINK_LIBRARIES(SelectPixel vtkHybrid)


