cmake_minimum_required(VERSION 2.6)

PROJECT(MovableAxes)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MovableAxes MovableAxes.cxx)
TARGET_LINK_LIBRARIES(MovableAxes vtkHybrid)
