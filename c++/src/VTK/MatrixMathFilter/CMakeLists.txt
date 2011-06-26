cmake_minimum_required(VERSION 2.6)
PROJECT(MatrixMathFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MatrixMathFilter MatrixMathFilter.cxx)
TARGET_LINK_LIBRARIES(MatrixMathFilter vtkHybrid)

