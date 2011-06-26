cmake_minimum_required(VERSION 2.6)

PROJECT(MarchingCubes)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MarchingCubes MarchingCubes.cxx)

TARGET_LINK_LIBRARIES(MarchingCubes vtkHybrid)
