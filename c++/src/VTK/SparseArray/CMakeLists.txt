cmake_minimum_required(VERSION 2.6)

PROJECT(SparseArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SparseArray SparseArray.cxx)
TARGET_LINK_LIBRARIES(SparseArray vtkHybrid)
