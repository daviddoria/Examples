cmake_minimum_required(VERSION 2.6)
PROJECT(PolyDataNormals)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PolyDataNormals PolyDataNormals.cxx)
TARGET_LINK_LIBRARIES(PolyDataNormals vtkHybrid)

ADD_EXECUTABLE(Input Input.cxx)
TARGET_LINK_LIBRARIES(Input vtkHybrid)

