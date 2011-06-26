cmake_minimum_required(VERSION 2.6)
PROJECT(vtkAlgorithmSourceMultiplePortsExample)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(vtkAlgorithmSourceMultiplePortsExample vtkAlgorithmSourceMultiplePortsExample.cxx 
vtkTestA.cxx 
vtkTestB.cxx
vtkTestSource.cxx
)
TARGET_LINK_LIBRARIES(vtkAlgorithmSourceMultiplePortsExample vtkHybrid)

