cmake_minimum_required(VERSION 2.6)

PROJECT(XMLPUnstructuredGridWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(XMLPUnstructuredGridWriter XMLPUnstructuredGridWriter.cxx)
TARGET_LINK_LIBRARIES(XMLPUnstructuredGridWriter vtkHybrid)
