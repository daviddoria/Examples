cmake_minimum_required(VERSION 2.6)

PROJECT(DelimitedTextWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DelimitedTextWriter DelimitedTextWriter.cxx)
TARGET_LINK_LIBRARIES(DelimitedTextWriter vtkHybrid vtkInfovis)
