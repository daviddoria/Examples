cmake_minimum_required(VERSION 2.6)
PROJECT(FieldData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(FieldData FieldData.cxx)
TARGET_LINK_LIBRARIES(FieldData vtkHybrid)

