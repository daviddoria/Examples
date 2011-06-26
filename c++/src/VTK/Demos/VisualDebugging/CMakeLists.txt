cmake_minimum_required(VERSION 2.6)

PROJECT(VisualDebugging)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VisualDebugging VisualDebugging.cxx vtkTestFilter.cxx)

TARGET_LINK_LIBRARIES(VisualDebugging vtkHybrid)
