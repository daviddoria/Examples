cmake_minimum_required(VERSION 2.6)
PROJECT(Casting)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Casting Casting.cxx)
TARGET_LINK_LIBRARIES(Casting vtkHybrid)

ADD_EXECUTABLE(Casting2 Casting2.cxx)
TARGET_LINK_LIBRARIES(Casting2 vtkHybrid)

