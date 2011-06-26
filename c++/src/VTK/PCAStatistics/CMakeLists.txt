cmake_minimum_required(VERSION 2.6)

PROJECT(PCAStatistics)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PCAStatistics PCAStatistics.cxx)
TARGET_LINK_LIBRARIES(PCAStatistics vtkHybrid vtkInfovis)
