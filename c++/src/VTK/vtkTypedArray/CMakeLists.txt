cmake_minimum_required(VERSION 2.6)
PROJECT(TypedArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TypedArray TypedArray.cxx)
TARGET_LINK_LIBRARIES(TypedArray vtkHybrid)

