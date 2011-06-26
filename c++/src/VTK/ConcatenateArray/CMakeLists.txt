cmake_minimum_required(VERSION 2.6)

PROJECT(ConcatenateArray)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConcatenateArray ConcatenateArray.cxx)
TARGET_LINK_LIBRARIES(ConcatenateArray vtkHybrid vtkTextAnalysis)
