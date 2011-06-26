cmake_minimum_required(VERSION 2.6)

PROJECT(EarthSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EarthSource EarthSource.cxx)

TARGET_LINK_LIBRARIES(EarthSource vtkHybrid)
