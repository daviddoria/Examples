cmake_minimum_required(VERSION 2.6)
PROJECT(AlgorithmFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AlgorithmFilter Example.cxx vtkAlgorithmFilter.cxx)
TARGET_LINK_LIBRARIES(AlgorithmFilter vtkHybrid)

