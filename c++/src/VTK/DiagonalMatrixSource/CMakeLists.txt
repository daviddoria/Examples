cmake_minimum_required(VERSION 2.6)

PROJECT(DiagonalMatrixSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DiagonalMatrixSource DiagonalMatrixSource.cxx)

TARGET_LINK_LIBRARIES(DiagonalMatrixSource vtkHybrid vtkInfovis)
