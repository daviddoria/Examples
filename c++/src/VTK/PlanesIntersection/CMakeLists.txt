cmake_minimum_required(VERSION 2.6)
PROJECT(PlanesIntersection)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PlanesIntersection PlanesIntersection.cxx)
TARGET_LINK_LIBRARIES(PlanesIntersection vtkHybrid)

