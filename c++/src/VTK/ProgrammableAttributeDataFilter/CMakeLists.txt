cmake_minimum_required(VERSION 2.6)
PROJECT(ProgrammableAttributeDataFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ProgrammableAttributeDataFilter ProgrammableAttributeDataFilter.cxx)
TARGET_LINK_LIBRARIES(ProgrammableAttributeDataFilter vtkHybrid)

