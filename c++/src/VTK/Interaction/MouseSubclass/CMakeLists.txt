cmake_minimum_required(VERSION 2.6)

PROJECT(MouseSubclass)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MouseSubclass MouseSubclass.cxx)
TARGET_LINK_LIBRARIES(MouseSubclass vtkHybrid)

