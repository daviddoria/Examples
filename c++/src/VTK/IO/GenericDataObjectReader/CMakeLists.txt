cmake_minimum_required(VERSION 2.6)

PROJECT(GenericDataObjectReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GenericDataObjectReader GenericDataObjectReader.cxx)
TARGET_LINK_LIBRARIES(GenericDataObjectReader vtkHybrid)
