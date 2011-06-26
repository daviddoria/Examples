cmake_minimum_required(VERSION 2.6)
PROJECT(PolygonalSurfacePointPlacer)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PolygonalSurfacePointPlacer PolygonalSurfacePointPlacer.cxx)
TARGET_LINK_LIBRARIES(PolygonalSurfacePointPlacer vtkHybrid vtkWidgets)

