cmake_minimum_required(VERSION 2.6)

PROJECT(IncrementalOctreePointLocator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(IncrementalOctreePointLocator IncrementalOctreePointLocator.cxx)
TARGET_LINK_LIBRARIES(IncrementalOctreePointLocator vtkHybrid)

