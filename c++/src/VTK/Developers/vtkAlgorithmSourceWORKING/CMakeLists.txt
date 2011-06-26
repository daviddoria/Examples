cmake_minimum_required(VERSION 2.6)
PROJECT(vtkAlgorithmSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(vtkAlgorithmSource Example.cxx vtkTest.cxx vtkTestSource.cxx vtkTestAlgorithm.cxx)
TARGET_LINK_LIBRARIES(vtkAlgorithmSource vtkHybrid)

