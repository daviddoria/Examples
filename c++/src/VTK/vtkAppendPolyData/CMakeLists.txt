cmake_minimum_required(VERSION 2.6)
PROJECT(AppendPolyData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AppendPolyData AppendPolyData.cxx)
TARGET_LINK_LIBRARIES(AppendPolyData vtkHybrid)

