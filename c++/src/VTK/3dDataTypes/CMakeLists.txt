cmake_minimum_required(VERSION 2.6)

PROJECT(3dDataTypes)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(3dDataTypes 3dDataTypes.cxx)
TARGET_LINK_LIBRARIES(3dDataTypes vtkHybrid)
