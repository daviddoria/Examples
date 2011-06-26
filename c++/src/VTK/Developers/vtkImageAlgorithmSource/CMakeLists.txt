cmake_minimum_required(VERSION 2.6)
PROJECT(vtkImageAlgorithmSourceExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(vtkImageAlgorithmSourceExample vtkImageAlgorithmSourceExample.cxx vtkImageAlgorithmSource.cxx)
TARGET_LINK_LIBRARIES(vtkImageAlgorithmSourceExample vtkHybrid)

