cmake_minimum_required(VERSION 2.6)
PROJECT(vtkPolyDataAlgorithmDemo)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ReaderExample ReaderExample.cxx vtkTestReader.cxx)
TARGET_LINK_LIBRARIES(ReaderExample vtkHybrid)

