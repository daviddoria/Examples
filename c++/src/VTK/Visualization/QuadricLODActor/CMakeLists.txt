cmake_minimum_required(VERSION 2.6)

PROJECT(QuadricLODActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(QuadricLODActor QuadricLODActor.cxx)
TARGET_LINK_LIBRARIES(QuadricLODActor vtkHybrid)
