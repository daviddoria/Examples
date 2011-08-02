cmake_minimum_required(VERSION 2.6)

PROJECT(OrderedTriangulator)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(OrderedTriangulator OrderedTriangulator.cxx)
TARGET_LINK_LIBRARIES(OrderedTriangulator ${VTK_LIBRARIES})
