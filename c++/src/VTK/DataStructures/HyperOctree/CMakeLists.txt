cmake_minimum_required(VERSION 2.6)

PROJECT(HyperOctree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(HyperOctree HyperOctree.cxx)
TARGET_LINK_LIBRARIES(HyperOctree vtkHybrid)
