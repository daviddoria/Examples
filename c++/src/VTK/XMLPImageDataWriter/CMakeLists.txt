cmake_minimum_required(VERSION 2.6)

PROJECT(XMLPImageDataWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(XMLPImageDataWriter XMLPImageDataWriter.cxx)
TARGET_LINK_LIBRARIES(XMLPImageDataWriter vtkHybrid)
