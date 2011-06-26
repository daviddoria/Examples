cmake_minimum_required(VERSION 2.6)
PROJECT(vtkImageAlgorithmFilterExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(vtkImageAlgorithmFilterExample vtkImageAlgorithmFilterExample.cxx vtkImageAlgorithmFilter.cxx)
TARGET_LINK_LIBRARIES(vtkImageAlgorithmFilterExample vtkHybrid)

