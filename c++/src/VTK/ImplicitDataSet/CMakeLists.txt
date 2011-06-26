cmake_minimum_required(VERSION 2.6)

PROJECT(ImplicitDataSet)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImplicitDataSet ImplicitDataSet.cxx)
TARGET_LINK_LIBRARIES(ImplicitDataSet vtkHybrid)
