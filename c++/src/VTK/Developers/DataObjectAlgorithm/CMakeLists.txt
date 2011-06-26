cmake_minimum_required(VERSION 2.6)
PROJECT(DataObjectAlgorithm)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DataObjectAlgorithm Example.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(DataObjectAlgorithm vtkHybrid)

