cmake_minimum_required(VERSION 2.6)

PROJECT(PickableOff)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PickableOff PickableOff.cxx)
TARGET_LINK_LIBRARIES(PickableOff vtkHybrid)
