cmake_minimum_required(VERSION 2.6)
PROJECT(vtkGraphAlgorithmSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SourceExample SourceExample.cxx vtkTestSource.cxx)
TARGET_LINK_LIBRARIES(SourceExample vtkHybrid)

