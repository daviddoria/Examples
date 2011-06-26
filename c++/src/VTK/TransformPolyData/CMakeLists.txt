cmake_minimum_required(VERSION 2.6)

PROJECT(TransformPolyData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TransformPolyData TransformPolyData.cxx)
TARGET_LINK_LIBRARIES(TransformPolyData vtkHybrid)
