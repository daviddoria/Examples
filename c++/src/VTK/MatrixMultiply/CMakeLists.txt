cmake_minimum_required(VERSION 2.6)
PROJECT(MatrixMultiply)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MatrixMultiply MatrixMultiply.cxx)
TARGET_LINK_LIBRARIES(MatrixMultiply vtkHybrid)

