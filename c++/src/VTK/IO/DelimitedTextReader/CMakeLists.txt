cmake_minimum_required(VERSION 2.6)

PROJECT(DelimitedTextReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(DelimitedTextReader DelimitedTextReader.cxx)
TARGET_LINK_LIBRARIES(DelimitedTextReader vtkHybrid vtkInfovis)
