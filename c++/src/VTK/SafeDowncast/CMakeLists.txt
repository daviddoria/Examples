cmake_minimum_required(VERSION 2.6)

PROJECT(SafeDowncast)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SafeDowncast SafeDowncast.cpp)
TARGET_LINK_LIBRARIES(SafeDowncast vtkHybrid)
