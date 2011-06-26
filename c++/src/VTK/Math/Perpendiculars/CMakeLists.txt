cmake_minimum_required(VERSION 2.6)

PROJECT(Perpendiculars)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Perpendiculars Perpendiculars.cxx)
TARGET_LINK_LIBRARIES(Perpendiculars vtkHybrid)
