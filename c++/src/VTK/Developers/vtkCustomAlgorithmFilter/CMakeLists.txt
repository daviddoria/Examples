cmake_minimum_required(VERSION 2.6)
PROJECT(vtkAlgorithmFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(vtkAlgorithmFilter Example.cxx vtkTest.cxx vtkTestFilter.cxx)
TARGET_LINK_LIBRARIES(vtkAlgorithmFilter vtkHybrid)

