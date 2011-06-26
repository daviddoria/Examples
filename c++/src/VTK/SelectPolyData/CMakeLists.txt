cmake_minimum_required(VERSION 2.6)
PROJECT(SelectPolyData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectPolyData SelectPolyData.cxx)
TARGET_LINK_LIBRARIES(SelectPolyData vtkHybrid)

