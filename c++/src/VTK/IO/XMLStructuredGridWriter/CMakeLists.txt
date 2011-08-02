cmake_minimum_required(VERSION 2.6)

PROJECT(XMLStructuredGridWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(XMLStructuredGridWriter XMLStructuredGridWriter.cxx)
TARGET_LINK_LIBRARIES(XMLStructuredGridWriter ${VTK_LIBRARIES})
