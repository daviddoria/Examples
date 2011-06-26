cmake_minimum_required(VERSION 2.6)
PROJECT(CleanPolyData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CleanPolyData CleanPolyData.cxx)
TARGET_LINK_LIBRARIES(CleanPolyData vtkHybrid)

