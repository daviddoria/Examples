cmake_minimum_required(VERSION 2.6)

PROJECT(TransposeMatrix)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TransposeMatrix TransposeMatrix.cxx)
TARGET_LINK_LIBRARIES(TransposeMatrix vtkHybrid vtkInfovis)
