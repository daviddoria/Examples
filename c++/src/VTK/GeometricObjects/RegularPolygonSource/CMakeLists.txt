cmake_minimum_required(VERSION 2.6)

PROJECT(RegularPolygonSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RegularPolygonSource RegularPolygonSource.cxx)

TARGET_LINK_LIBRARIES(RegularPolygonSource vtkHybrid)
