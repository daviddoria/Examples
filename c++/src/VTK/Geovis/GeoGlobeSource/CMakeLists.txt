cmake_minimum_required(VERSION 2.6)

PROJECT(GeoGlobeSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GeoGlobeSource GeoGlobeSource.cxx)

TARGET_LINK_LIBRARIES(GeoGlobeSource vtkHybrid vtkGeovis)
