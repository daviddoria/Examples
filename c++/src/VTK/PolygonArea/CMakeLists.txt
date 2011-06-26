cmake_minimum_required(VERSION 2.6)
 
PROJECT(PolygonArea)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(PolygonArea PolygonArea.cxx)
 
TARGET_LINK_LIBRARIES(PolygonArea vtkHybrid)
