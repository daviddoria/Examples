cmake_minimum_required(VERSION 2.6)
PROJECT(PointInPolygon)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointInPolygon PointInPolygon.cxx)
TARGET_LINK_LIBRARIES(PointInPolygon vtkHybrid)

