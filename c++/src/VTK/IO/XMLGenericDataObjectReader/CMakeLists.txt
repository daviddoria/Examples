cmake_minimum_required(VERSION 2.6)

PROJECT(XMLGenericDataObjectReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(XMLGenericDataObjectReader XMLGenericDataObjectReader.cxx)
TARGET_LINK_LIBRARIES(XMLGenericDataObjectReader vtkHybrid)
