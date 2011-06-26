cmake_minimum_required(VERSION 2.6)

PROJECT(GeoView)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GeoView GeoView.cxx)

TARGET_LINK_LIBRARIES(GeoView vtkHybrid vtkGeovis)
