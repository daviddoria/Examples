cmake_minimum_required(VERSION 2.6)

PROJECT(CustomDenseArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(CustomDenseArray CustomDenseArray.cxx)
TARGET_LINK_LIBRARIES(CustomDenseArray vtkHybrid)
